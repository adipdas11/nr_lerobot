#!/usr/bin/env python3
"""Convert wooden-block rosbags → LeRobot v3.0 dataset format for ACT training.

Key differences from lerobot_pick_place/act/rosbag_to_lerobot_act.py:
  - Joint topic: /isaac_joint_states  (Isaac Sim direct bridge)
  - Task description: stack wooden blocks
  - Idle-frame trimming: strips stationary frames at start/end of each episode
    so the model does not learn "stay still" from the home/reset phase.

Usage:
    python rosbag_to_lerobot_act.py \\
        --dir rosbags \\
        --output lerobot_dataset/wooden_block_act \\
        --fps 30

Train:
    python -m lerobot.scripts.lerobot_train \\
        --policy.type act \\
        --dataset.repo_id wooden_block_act \\
        --dataset.root lerobot_dataset/wooden_block_act \\
        --dataset.use_imagenet_stats true \\
        --wandb.enable false \\
        --output_dir training_output \\
        --policy.push_to_hub false \\
        --batch_size 8 --num_workers 4
"""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_BAGS_DIR = SCRIPT_DIR / "rosbags"
DEFAULT_OUTPUT_ROOT = SCRIPT_DIR / "lerobot_dataset"

DEFAULT_ARM_JOINTS = [
    "xarm5_joint1", "xarm5_joint2", "xarm5_joint3",
    "xarm5_joint4", "xarm5_joint5",
]
DEFAULT_GRIPPER_JOINT = "xarm_gripper_right_drive_joint"


# ── rosbag helpers ────────────────────────────────────────────────────────────

def decode_joint_state(rawdata, typestore):
    msg = typestore.deserialize_cdr(rawdata, "sensor_msgs/msg/JointState")
    return {
        "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "names":     list(msg.name),
        "positions": list(msg.position),
    }


def decode_compressed_image(rawdata, typestore):
    msg = typestore.deserialize_cdr(rawdata, "sensor_msgs/msg/CompressedImage")
    ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    arr = np.frombuffer(msg.data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return {"timestamp": ts, "image": img}


def read_rosbag(bag_path, joint_topic, camera_topics, typestore):
    joint_states = []
    camera_frames = {t: [] for t in camera_topics}
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        wanted = set([joint_topic] + camera_topics)
        conns = [c for c in reader.connections if c.topic in wanted]
        for conn, _ts, raw in reader.messages(connections=conns):
            if conn.topic == joint_topic:
                joint_states.append(decode_joint_state(raw, typestore))
            elif conn.topic in camera_frames:
                camera_frames[conn.topic].append(decode_compressed_image(raw, typestore))
    return joint_states, camera_frames


def synchronize_data(
    joint_states,
    camera_frames,
    fps,
    arm_joint_names,
    gripper_joint_name=None,
    default_gripper=0.0,
    trim_idle_threshold: float = 0.005,
    trim_idle_frames: int = 5,
):
    """Interpolate all streams to a uniform fps grid and optionally trim idle frames."""
    all_ts = [js["timestamp"] for js in joint_states]
    for frames in camera_frames.values():
        all_ts.extend(f["timestamp"] for f in frames)
    t_start, t_end = min(all_ts), max(all_ts)

    n_frames = max(1, int(round((t_end - t_start) * fps)))
    timeline = t_start + np.arange(n_frames) / fps

    state_joints = list(arm_joint_names)
    if gripper_joint_name:
        state_joints.append(gripper_joint_name)

    filtered = []
    for js in joint_states:
        nm = dict(zip(js["names"], js["positions"]))
        if not all(n in nm for n in arm_joint_names):
            continue
        row = [float(nm[n]) for n in arm_joint_names]
        if gripper_joint_name:
            row.append(float(nm.get(gripper_joint_name, default_gripper)))
        filtered.append((js["timestamp"], row))

    if not filtered:
        raise ValueError(f"No JointState messages had all required joints: {arm_joint_names}")

    js_t = np.array([r[0] for r in filtered], dtype=np.float64)
    js_p = np.array([r[1] for r in filtered], dtype=np.float32)

    synced_pos = np.zeros((n_frames, len(state_joints)), dtype=np.float32)
    for j in range(len(state_joints)):
        synced_pos[:, j] = np.interp(timeline, js_t, js_p[:, j])

    synced_imgs = {}
    for topic, frames in camera_frames.items():
        cam_t = np.array([f["timestamp"] for f in frames])
        idx = np.clip(np.searchsorted(cam_t, timeline, side="right") - 1, 0, len(frames) - 1)
        synced_imgs[topic] = [frames[i]["image"] for i in idx]

    # ── idle trimming ────────────────────────────────────────────────────────
    n_arm = len(arm_joint_names)
    start_cut = 0
    if trim_idle_threshold > 0 and n_frames > trim_idle_frames:
        ref = synced_pos[0, :n_arm]
        for fi in range(1, n_frames):
            if np.abs(synced_pos[fi, :n_arm] - ref).max() >= trim_idle_threshold:
                start_cut = max(0, fi - trim_idle_frames)
                break

    end_cut = n_frames
    if trim_idle_threshold > 0 and n_frames > trim_idle_frames:
        ref = synced_pos[-1, :n_arm]
        for fi in range(n_frames - 2, -1, -1):
            if np.abs(synced_pos[fi, :n_arm] - ref).max() >= trim_idle_threshold:
                end_cut = min(n_frames, fi + 1 + trim_idle_frames)
                break

    if start_cut > 0 or end_cut < n_frames:
        synced_pos = synced_pos[start_cut:end_cut]
        timeline = timeline[start_cut:end_cut]
        for topic in synced_imgs:
            synced_imgs[topic] = synced_imgs[topic][start_cut:end_cut]
        n_frames = end_cut - start_cut
        print(f"  Idle trim: [{start_cut}, {end_cut}) → {n_frames} frames")

    return (timeline - timeline[0]).astype(np.float32), synced_pos, synced_imgs


def get_video_frame_count(video_path: str) -> int:
    result = subprocess.run(
        ["ffprobe", "-v", "quiet", "-print_format", "json", "-show_streams", video_path],
        capture_output=True, text=True,
    )
    info = json.loads(result.stdout)
    for s in info.get("streams", []):
        if s.get("codec_type") == "video":
            return int(s.get("nb_frames", 0))
    return 0


def write_video(frames, output_path: Path, fps: int) -> int:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    h, w = frames[0].shape[:2]
    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h)
    )
    for frame in frames:
        writer.write(frame)
    writer.release()
    return len(frames)


# ── conversion ────────────────────────────────────────────────────────────────

def convert_bags(
    bag_paths,
    output_dir,
    fps,
    task_description,
    joint_topic,
    camera_topics,
    camera_names,
    arm_joint_names,
    gripper_joint_name,
    default_gripper=0.0,
    trim_idle_threshold=0.005,
    trim_idle_frames=5,
):
    output = Path(output_dir)
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    state_dim = len(arm_joint_names) + 1

    global_index = 0
    episode_records = []
    all_states, all_actions = [], []
    valid_episodes = []

    for ep_idx, bag_path in enumerate(bag_paths):
        print(f"\n[Episode {ep_idx}] {Path(bag_path).name}")
        joint_states, camera_frames = read_rosbag(bag_path, joint_topic, camera_topics, typestore)
        if not joint_states:
            print("  SKIP: no joint states")
            continue

        try:
            timestamps, states, images = synchronize_data(
                joint_states, camera_frames, fps,
                arm_joint_names=arm_joint_names,
                gripper_joint_name=gripper_joint_name,
                default_gripper=default_gripper,
                trim_idle_threshold=trim_idle_threshold,
                trim_idle_frames=trim_idle_frames,
            )
        except ValueError as exc:
            print(f"  SKIP: {exc}")
            continue

        n_frames = len(timestamps)

        # Action = next absolute joint position
        actions = np.roll(states, -1, axis=0)
        actions[-1] = states[-1]

        chunk_idx = ep_idx // 1000
        chunk = f"chunk-{chunk_idx:03d}"
        file_idx = ep_idx % 1000

        video_frame_counts = {}
        for topic, cam_name in zip(camera_topics, camera_names):
            video_path = (
                output / "videos" / chunk
                / f"observation.images.{cam_name}"
                / f"episode_{ep_idx:06d}.mp4"
            )
            write_video(images[topic], video_path, fps)
            count = get_video_frame_count(str(video_path))
            video_frame_counts[cam_name] = count
            print(f"  [{cam_name}] {count} frames")

        safe_n = min(n_frames, *video_frame_counts.values())
        if safe_n < n_frames:
            print(f"  Trim {n_frames} → {safe_n} frames (video length)")
            n_frames = safe_n
            timestamps = timestamps[:n_frames]
            states = states[:n_frames]
            actions = actions[:n_frames]

        parquet_dir = output / "data" / chunk
        parquet_dir.mkdir(parents=True, exist_ok=True)
        table = pa.table({
            "episode_index": pa.array(np.full(n_frames, ep_idx, dtype=np.int64)),
            "frame_index":   pa.array(np.arange(n_frames, dtype=np.int64)),
            "index":         pa.array(np.arange(global_index, global_index + n_frames, dtype=np.int64)),
            "timestamp":     pa.array(timestamps),
            "task_index":    pa.array(np.zeros(n_frames, dtype=np.int64)),
            "observation.state": pa.array(
                [states[i].tolist() for i in range(n_frames)], type=pa.list_(pa.float32())
            ),
            "action": pa.array(
                [actions[i].tolist() for i in range(n_frames)], type=pa.list_(pa.float32())
            ),
        })
        pq.write_table(table, parquet_dir / f"episode_{ep_idx:06d}.parquet")

        ep_record = {
            "episode_index":      ep_idx,
            "tasks":              [task_description],
            "length":             n_frames,
            "dataset_from_index": global_index,
            "dataset_to_index":   global_index + n_frames,
            "data/chunk_index":   chunk_idx,
            "data/file_index":    file_idx,
        }
        for cam_name in camera_names:
            ep_record[f"videos/observation.images.{cam_name}/chunk_index"] = chunk_idx
            ep_record[f"videos/observation.images.{cam_name}/file_index"]  = file_idx
            ep_record[f"videos/observation.images.{cam_name}/from_timestamp"] = 0.0
            ep_record[f"videos/observation.images.{cam_name}/to_timestamp"] = (
                (video_frame_counts[cam_name] - 1.01) / fps
            )

        episode_records.append(ep_record)
        valid_episodes.append(ep_idx)
        all_states.append(states)
        all_actions.append(actions)
        global_index += n_frames

    if not valid_episodes:
        raise RuntimeError("No valid episodes converted.")

    total_episodes = len(valid_episodes)
    total_frames = global_index
    state_names = [f"joint{i+1}" for i in range(len(arm_joint_names))] + ["gripper"]

    meta_dir = output / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    info = {
        "codebase_version": "v3.0",
        "robot_type":       "xarm5",
        "fps":              fps,
        "total_episodes":   total_episodes,
        "total_frames":     total_frames,
        "total_tasks":      1,
        "total_videos":     total_episodes * len(camera_names),
        "total_chunks":     max(1, (total_episodes - 1) // 1000 + 1),
        "chunks_size":      1000,
        "splits":           {"train": f"0:{total_episodes}"},
        "data_path":  "data/chunk-{chunk_index:03d}/episode_{file_index:06d}.parquet",
        "video_path": "videos/chunk-{chunk_index:03d}/{video_key}/episode_{file_index:06d}.mp4",
        "features": {
            "episode_index": {"dtype": "int64",   "shape": [1], "names": None},
            "frame_index":   {"dtype": "int64",   "shape": [1], "names": None},
            "timestamp":     {"dtype": "float32", "shape": [1], "names": None},
            "index":         {"dtype": "int64",   "shape": [1], "names": None},
            "task_index":    {"dtype": "int64",   "shape": [1], "names": None},
            "observation.state": {"dtype": "float32", "shape": [state_dim], "names": state_names},
            "action":            {"dtype": "float32", "shape": [state_dim], "names": state_names},
            **{
                f"observation.images.{cam}": {
                    "dtype": "video",
                    "shape": [480, 640, 3],
                    "names": ["height", "width", "channels"],
                    "info": {
                        "video.fps": float(fps), "video.codec": "mp4v",
                        "video.pix_fmt": "yuv420p", "video.is_depth_map": False,
                        "has_audio": False,
                    },
                }
                for cam in camera_names
            },
        },
    }
    with open(meta_dir / "info.json", "w") as f:
        json.dump(info, f, indent=2)

    tasks_df = pd.DataFrame([{"task": task_description, "task_index": 0}]).set_index("task")
    pq.write_table(pa.Table.from_pandas(tasks_df), str(meta_dir / "tasks.parquet"))

    ep_chunk_dir = meta_dir / "episodes" / "chunk-000"
    ep_chunk_dir.mkdir(parents=True, exist_ok=True)
    pq.write_table(
        pa.Table.from_pandas(pd.DataFrame(episode_records), preserve_index=False),
        str(ep_chunk_dir / "episodes.parquet"),
    )

    all_s = np.vstack(all_states)
    all_a = np.vstack(all_actions)
    stats = {}
    for key, arr in [("observation.state", all_s), ("action", all_a)]:
        stats[key] = {
            "mean": arr.mean(axis=0).tolist(),
            "std":  np.maximum(arr.std(axis=0), 1e-6).tolist(),
            "min":  arr.min(axis=0).tolist(),
            "max":  arr.max(axis=0).tolist(),
        }
    for cam in camera_names:
        stats[f"observation.images.{cam}"] = {
            "mean": [0.485, 0.456, 0.406],
            "std":  [0.229, 0.224, 0.225],
        }
    with open(meta_dir / "stats.json", "w") as f:
        json.dump(stats, f, indent=2)

    print(f"\n✓ Dataset written → {output}")
    print(f"  Episodes : {total_episodes}")
    print(f"  Frames   : {total_frames}")
    print(f"  State dim: {state_dim}  {state_names}")
    print(f"  Cameras  : {camera_names}")


# ── validation ────────────────────────────────────────────────────────────────

def validate_dataset(dataset_path: str) -> bool:
    p = Path(dataset_path)
    errors = []
    for req in [
        p / "meta" / "info.json", p / "meta" / "stats.json",
        p / "meta" / "tasks.parquet",
        p / "meta" / "episodes" / "chunk-000" / "episodes.parquet",
    ]:
        if not req.exists():
            errors.append(f"Missing: {req}")

    data_files  = list((p / "data").rglob("*.parquet"))  if (p / "data").exists()   else []
    video_files = list((p / "videos").rglob("*.mp4"))    if (p / "videos").exists() else []
    if not data_files:
        errors.append("No parquet files under data/")
    if not video_files:
        errors.append("No MP4 files under videos/")

    if errors:
        print("[Validate] ERRORS:")
        for e in errors:
            print(f"  ✗ {e}")
        return False

    with open(p / "meta" / "info.json") as f:
        info = json.load(f)
    print(f"[Validate] ✓  episodes={info['total_episodes']}  frames={info['total_frames']}  fps={info['fps']}")
    return True


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Convert wooden-block rosbags → LeRobot v3.0 ACT dataset"
    )
    parser.add_argument("--dir",          default=str(DEFAULT_BAGS_DIR))
    parser.add_argument("--output-root",  default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--output",       default=None)
    parser.add_argument("--fps",          type=int,   default=30)
    parser.add_argument("--task",         default="stack the wooden blocks with block 1 at the bottom block 2 in the middle and block 3 on top")
    parser.add_argument("--joint-topic",  default="/isaac_joint_states")
    parser.add_argument("--arm-joint-names", nargs="+", default=DEFAULT_ARM_JOINTS)
    parser.add_argument("--gripper-joint",   default=DEFAULT_GRIPPER_JOINT)
    parser.add_argument("--camera1-topic", default="/camera1/realsense_camera/color/image_raw/compressed")
    parser.add_argument("--camera2-topic", default="/camera2/realsense_camera/color/image_raw/compressed")
    parser.add_argument("--trim-idle-threshold", type=float, default=0.005,
                        help="Joint motion threshold (rad) to detect idle frames")
    parser.add_argument("--trim-idle-frames",    type=int,   default=5,
                        help="Context frames to keep around the trim cut points")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()

    if args.output:
        output_dir = args.output
    else:
        dataset_name = Path(args.dir).name.replace("rosbags", "") or "wooden_block_act"
        output_dir = str(Path(args.output_root) / "wooden_block_act")

    if args.validate_only:
        ok = validate_dataset(output_dir)
        return 0 if ok else 1

    bag_paths = sorted(
        [str(p) for p in Path(args.dir).iterdir() if p.is_dir()],
    )
    if not bag_paths:
        print(f"No bag directories found in {args.dir}", file=__import__("sys").stderr)
        return 1

    print(f"Found {len(bag_paths)} bags in {args.dir}")
    convert_bags(
        bag_paths        = bag_paths,
        output_dir       = output_dir,
        fps              = args.fps,
        task_description = args.task,
        joint_topic      = args.joint_topic,
        camera_topics    = [args.camera1_topic, args.camera2_topic],
        camera_names     = ["camera1", "camera2"],
        arm_joint_names  = args.arm_joint_names,
        gripper_joint_name = args.gripper_joint,
        trim_idle_threshold = args.trim_idle_threshold,
        trim_idle_frames    = args.trim_idle_frames,
    )

    validate_dataset(output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
