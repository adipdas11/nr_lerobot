#!/usr/bin/env python3
"""Python 3.12 ACT inference server.

Loads a trained ACT checkpoint and serves action-chunk predictions over a
Unix-domain socket. The ROS-side client (act_ros_client.py) connects here to
send observations and receive action chunks.

ACT differences from SmolVLA:
  - No language task string — ACT is not language-conditioned.
  - No VLM model to resolve from HuggingFace cache.
  - Typically larger chunk_size (100 steps default).
  - Faster inference (~50-100 ms vs ~500-1000 ms for SmolVLA).
"""

from __future__ import annotations

import argparse
import socket
import sys
from pathlib import Path

import cv2
import numpy as np
import torch

from act_ipc import recv_message, send_message


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CHECKPOINT_ROOT = SCRIPT_DIR / "training_output"
DEFAULT_SOCKET_PATH = "/tmp/act_policy.sock"


def find_latest_model_dir(checkpoint_root: Path) -> Path:
    """Find the highest-numbered pretrained_model directory under checkpoint_root."""
    direct = checkpoint_root / "pretrained_model"
    if (direct / "config.json").is_file():
        return direct

    candidates: list[tuple[int, Path]] = []

    def collect(parent: Path) -> None:
        if not parent.is_dir():
            return
        for child in parent.iterdir():
            if child.is_dir() and child.name.isdigit():
                model_dir = child / "pretrained_model"
                if (model_dir / "config.json").is_file():
                    candidates.append((int(child.name), model_dir))

    collect(checkpoint_root)
    collect(checkpoint_root / "checkpoints")

    if not candidates:
        raise FileNotFoundError(f"No pretrained_model found under {checkpoint_root}")
    return max(candidates, key=lambda x: x[0])[1]


def resolve_device(device_arg: str) -> str:
    if device_arg == "auto":
        return "cuda" if torch.cuda.is_available() else "cpu"
    return device_arg


class ACTPolicyServer:
    def __init__(self, args: argparse.Namespace) -> None:
        self._socket_path = Path(args.socket_path)
        self._model_dir = (
            Path(args.model_dir).expanduser().resolve()
            if args.model_dir
            else find_latest_model_dir(Path(args.checkpoint_root).expanduser().resolve())
        )
        self._policy, self._preprocess, self._postprocess, self._cfg = self._load_policy(
            self._model_dir,
            resolve_device(args.device),
        )
        self._policy.reset()

    def _load_policy(self, model_dir: Path, device: str):
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors

        cfg = PreTrainedConfig.from_pretrained(model_dir)
        if cfg.type != "act":
            print(
                f"WARNING: checkpoint type is '{cfg.type}', expected 'act'. "
                "Proceeding anyway.",
                file=sys.stderr,
            )
        cfg.device = device
        policy_cls = get_policy_class(cfg.type)
        policy = policy_cls.from_pretrained(model_dir, config=cfg)
        policy.to(device)
        policy.eval()
        preprocess, postprocess = make_pre_post_processors(
            cfg,
            str(model_dir),
            preprocessor_overrides={"device_processor": {"device": device}},
        )
        print(f"[ACT] Loaded checkpoint: {model_dir}", flush=True)
        print(f"[ACT] Device: {device}", flush=True)
        print(
            f"[ACT] chunk_size={cfg.chunk_size}  n_action_steps={cfg.n_action_steps}",
            flush=True,
        )
        return policy, preprocess, postprocess, cfg

    def _decode_image(self, encoded: bytes) -> np.ndarray:
        image = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Failed to decode compressed image bytes")
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    def predict(self, request: dict) -> dict:
        from lerobot.policies.utils import build_inference_frame

        state = np.asarray(request["state"], dtype=np.float32)
        if state.shape != (6,):
            raise ValueError(f"Expected state shape (6,), got {state.shape}")

        image1 = self._decode_image(request["camera1"])
        image2 = self._decode_image(request["camera2"])

        ds_features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (6,),
                "names": ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper"],
            },
            "observation.images.camera1": {
                "dtype": "image",
                "shape": image1.shape,
                "names": ["height", "width", "channels"],
            },
            "observation.images.camera2": {
                "dtype": "image",
                "shape": image2.shape,
                "names": ["height", "width", "channels"],
            },
            "action": {
                "dtype": "float32",
                "shape": (6,),
                "names": [
                    "xarm5_joint1", "xarm5_joint2", "xarm5_joint3",
                    "xarm5_joint4", "xarm5_joint5",
                    "xarm_gripper_right_drive_joint",
                ],
            },
        }

        observation = {
            "joint1": state[0],
            "joint2": state[1],
            "joint3": state[2],
            "joint4": state[3],
            "joint5": state[4],
            "gripper": state[5],
            "camera1": image1,
            "camera2": image2,
        }

        # ACT does not use a task string — pass None / omit.
        obs_frame = build_inference_frame(
            observation=observation,
            ds_features=ds_features,
            device=self._cfg.device,
            robot_type="xarm5",
        )
        obs_batch = self._preprocess(obs_frame)
        action_chunk = self._policy.predict_action_chunk(obs_batch)
        action_chunk = self._postprocess(action_chunk)
        actions = action_chunk.squeeze(0).detach().cpu().numpy()
        return {
            "ok": True,
            "actions": actions.tolist(),
            "shape": list(actions.shape),
            "model_dir": str(self._model_dir),
        }

    def serve_forever(self) -> None:
        if self._socket_path.exists():
            self._socket_path.unlink()
        self._socket_path.parent.mkdir(parents=True, exist_ok=True)

        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(self._socket_path))
            server.listen(1)
            print(f"[ACT] Server ready on {self._socket_path}", flush=True)

            while True:
                conn, _ = server.accept()
                with conn:
                    try:
                        request = recv_message(conn)
                        response = self.predict(request)
                    except Exception as exc:
                        response = {"ok": False, "error": str(exc)}
                    send_message(conn, response)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ACT inference server")
    parser.add_argument("--checkpoint-root", default=str(DEFAULT_CHECKPOINT_ROOT),
                        help="Root directory to search for checkpoints (default: act/training_output)")
    parser.add_argument("--model-dir", default=None,
                        help="Direct path to a pretrained_model directory (overrides --checkpoint-root)")
    parser.add_argument("--socket-path", default=DEFAULT_SOCKET_PATH)
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda"])
    return parser


def main() -> int:
    args, _unknown = build_arg_parser().parse_known_args()
    try:
        ACTPolicyServer(args).serve_forever()
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"Failed to start ACT policy server: {exc}", file=sys.stderr)
        return 1
    finally:
        socket_path = Path(args.socket_path)
        if socket_path.exists():
            socket_path.unlink()


if __name__ == "__main__":
    raise SystemExit(main())
