#!/usr/bin/env python3
"""Python 3.12 SmolVLA inference server.

This process owns the LeRobot/SmolVLA runtime and communicates over a local
Unix-domain socket with the ROS-side Python 3.10 relay.
"""

from __future__ import annotations

import argparse
import socket
import sys
from pathlib import Path

import cv2
import numpy as np
import torch

from smolvla_ipc import recv_message, send_message


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CHECKPOINT_ROOT = SCRIPT_DIR / "smolvla_training_output"
DEFAULT_SOCKET_PATH = "/tmp/smolvla_policy.sock"
DEFAULT_TASK = "pick up the cube"
HF_CACHE_ROOT = Path.home() / ".cache" / "huggingface" / "hub"


def find_latest_model_dir(checkpoint_root: Path) -> Path:
    direct_model_dir = checkpoint_root / "pretrained_model"
    if (direct_model_dir / "config.json").is_file():
        return direct_model_dir

    candidates: list[tuple[int, Path]] = []

    def collect_numeric_children(parent: Path) -> None:
        if not parent.is_dir():
            return
        for child in parent.iterdir():
            if child.is_dir() and child.name.isdigit():
                model_dir = child / "pretrained_model"
                if (model_dir / "config.json").is_file():
                    candidates.append((int(child.name), model_dir))

    # Support both historical flat layouts:
    #   smolvla_training_output/065000/pretrained_model
    # and current checkpoint layouts:
    #   smolvla_training_output/checkpoints/010000/pretrained_model
    collect_numeric_children(checkpoint_root)
    collect_numeric_children(checkpoint_root / "checkpoints")

    if not candidates:
        raise FileNotFoundError(f"No pretrained_model found under {checkpoint_root}")
    return max(candidates, key=lambda item: item[0])[1]


def resolve_device(device_arg: str) -> str:
    if device_arg == "auto":
        return "cuda" if torch.cuda.is_available() else "cpu"
    return device_arg


def resolve_local_vlm_snapshot(model_id: str) -> Path | None:
    model_cache_dir = HF_CACHE_ROOT / f"models--{model_id.replace('/', '--')}" / "snapshots"
    if not model_cache_dir.is_dir():
        return None

    candidates: list[Path] = []
    for child in model_cache_dir.iterdir():
        if child.is_dir() and (child / "processor_config.json").is_file():
            candidates.append(child)
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


class SmolVLAPolicyServer:
    def __init__(self, args: argparse.Namespace) -> None:
        self._socket_path = Path(args.socket_path)
        self._task = args.task
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
        cfg.device = device
        if getattr(cfg, "vlm_model_name", None):
            local_vlm_snapshot = resolve_local_vlm_snapshot(cfg.vlm_model_name)
            if local_vlm_snapshot is not None:
                cfg.vlm_model_name = str(local_vlm_snapshot)
        policy_cls = get_policy_class(cfg.type)
        policy = policy_cls.from_pretrained(model_dir, config=cfg)
        preprocess, postprocess = make_pre_post_processors(
            cfg,
            str(model_dir),
            preprocessor_overrides={"device_processor": {"device": device}},
        )
        return policy, preprocess, postprocess, cfg

    def _decode_image(self, encoded: bytes) -> np.ndarray:
        image = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Failed to decode compressed image")
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
                    "xarm5_joint1",
                    "xarm5_joint2",
                    "xarm5_joint3",
                    "xarm5_joint4",
                    "xarm5_joint5",
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

        obs_frame = build_inference_frame(
            observation=observation,
            ds_features=ds_features,
            device=self._cfg.device,
            task=request.get("task", self._task),
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
            print(f"SmolVLA server ready on {self._socket_path}", flush=True)
            print(f"SmolVLA checkpoint: {self._model_dir}", flush=True)

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
    parser = argparse.ArgumentParser(description="Run the SmolVLA inference server.")
    parser.add_argument("--checkpoint-root", default=str(DEFAULT_CHECKPOINT_ROOT))
    parser.add_argument("--model-dir", default=None)
    parser.add_argument("--socket-path", default=DEFAULT_SOCKET_PATH)
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda"])
    parser.add_argument("--task", default=DEFAULT_TASK)
    return parser


def main() -> int:
    args, _unknown = build_arg_parser().parse_known_args()
    try:
        SmolVLAPolicyServer(args).serve_forever()
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"Failed to start SmolVLA policy server: {exc}", file=sys.stderr)
        return 1
    finally:
        socket_path = Path(args.socket_path)
        if socket_path.exists():
            socket_path.unlink()


if __name__ == "__main__":
    raise SystemExit(main())
