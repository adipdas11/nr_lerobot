#!/usr/bin/env python3
"""
Convert ROS2 rosbags to LeRobot v3.0 dataset format (compatible with lerobot_train + smolVLA).

Key v3.0 requirements fixed vs old script:
  - info.json uses chunk_index / file_index path templates (not episode_chunk / episode_index)
  - info.json features must include: episode_index, frame_index, timestamp, index, task_index
  - episodes parquet must have columns:
      dataset_from_index, dataset_to_index,
      data/chunk_index, data/file_index,
      videos/<cam>/chunk_index, videos/<cam>/file_index,
      videos/<cam>/from_timestamp, videos/<cam>/to_timestamp
  - tasks.parquet must be indexed by 'task' (not 'task_index')
  - episodes/ directory must contain chunk-000/episodes.parquet (nested), not episodes.parquet directly
  - stats.json must exist at meta/stats.json (load_stats looks for JSON, not safetensors)
  - Data parquet must contain task_index column
  - Camera columns (image structs) must NOT be in the parquet - videos are resolved separately via meta
  - Video timestamps are 0-indexed from episode start; parquet timestamps must not exceed video duration

Usage:
    python rosbag_to_lerobot.py --dir /path/to/bags --output /path/to/dataset --task "pick up the cube"

Then train with:
    conda run -n lerobot python -m lerobot.scripts.lerobot_train \\
        --policy.type smolvla \\
        --dataset.repo_id my_dataset \\
        --dataset.root /path/to/dataset \\
        --dataset.use_imagenet_stats false \\
        --wandb.enable false \\
        --output_dir ./smolvla_output \\
        --policy.push_to_hub false \\
        --tolerance_s 0.05 \\
        --num_workers 0
"""

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
DEFAULT_OUTPUT_ROOT = SCRIPT_DIR / "lerobot_converted_dataset"


# ─────────────────────────── ROS bag helpers ────────────────────────────────

def decode_joint_state(rawdata, typestore):
    msg = typestore.deserialize_cdr(rawdata, "sensor_msgs/msg/JointState")
    return {
        "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "names": list(msg.name),
        "positions": list(msg.position),
    }


def decode_compressed_image(rawdata, typestore):
    msg = typestore.deserialize_cdr(rawdata, "sensor_msgs/msg/CompressedImage")
    timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    np_arr = np.frombuffer(msg.data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return {"timestamp": timestamp, "image": image}


def read_rosbag(bag_path, joint_topic, camera_topics, typestore):
    joint_states = []
    camera_frames = {topic: [] for topic in camera_topics}
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        all_topics = [joint_topic] + camera_topics
        connections = [c for c in reader.connections if c.topic in all_topics]
        for conn, _ts, rawdata in reader.messages(connections=connections):
            if conn.topic == joint_topic:
                joint_states.append(decode_joint_state(rawdata, typestore))
            elif conn.topic in camera_topics:
                camera_frames[conn.topic].append(decode_compressed_image(rawdata, typestore))
    return joint_states, camera_frames


def synchronize_data(
    joint_states,
    camera_frames,
    fps,
    arm_joint_names,
    gripper_joint_name=None,
    default_gripper=1.0,
):
    all_timestamps = [js["timestamp"] for js in joint_states]
    for frames in camera_frames.values():
        all_timestamps.extend(f["timestamp"] for f in frames)
    t_start = min(all_timestamps)
    t_end = max(all_timestamps)
    n_frames = int(round((t_end - t_start) * fps))
    if n_frames < 1: n_frames = 1
    # Use a slightly more robust timeline calculation
    timeline = t_start + np.arange(n_frames) / fps

    state_joint_names = list(arm_joint_names)
    if gripper_joint_name:
        state_joint_names.append(gripper_joint_name)

    filtered_js = []
    for js in joint_states:
        name_to_pos = dict(zip(js["names"], js["positions"]))
        if not all(name in name_to_pos for name in arm_joint_names):
            continue
        row = [float(name_to_pos[name]) for name in arm_joint_names]
        if gripper_joint_name:
            row.append(float(name_to_pos.get(gripper_joint_name, default_gripper)))
        filtered_js.append((js["timestamp"], row))

    if not filtered_js:
        raise ValueError(
            "No JointState messages contained all required arm joints: "
            f"{list(arm_joint_names)}"
        )

    js_times = np.array([item[0] for item in filtered_js], dtype=np.float64)
    js_positions = np.array([item[1] for item in filtered_js], dtype=np.float32)

    synced_positions = np.zeros((n_frames, len(state_joint_names)), dtype=np.float32)
    for j in range(len(state_joint_names)):
        synced_positions[:, j] = np.interp(timeline, js_times, js_positions[:, j])

    synced_images = {}
    for topic, frames in camera_frames.items():
        cam_times = np.array([f["timestamp"] for f in frames])
        indices = np.searchsorted(cam_times, timeline, side="right") - 1
        indices = np.clip(indices, 0, len(frames) - 1)
        synced_images[topic] = [frames[i]["image"] for i in indices]

    return (timeline - t_start).astype(np.float32), synced_positions, synced_images


def get_video_frame_count(video_path: str) -> int:
    """Use ffprobe to get the exact number of frames in a video."""
    result = subprocess.run(
        ["ffprobe", "-v", "quiet", "-print_format", "json", "-show_streams", video_path],
        capture_output=True, text=True,
    )
    info = json.loads(result.stdout)
    for s in info.get("streams", []):
        if s.get("codec_type") == "video":
            return int(s["nb_frames"])
    return 0


def write_video(frames, output_path: Path, fps: int):
    """Write frames to mp4. Returns the number of frames actually written."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    h, w = frames[0].shape[:2]
    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h)
    )
    for frame in frames:
        writer.write(frame)
    writer.release()
    return len(frames)


def resolve_output_dir(input_dir: str, output_root: str | None, output_dir: str | None) -> Path:
    """Resolve the final dataset directory from the input rosbag directory."""
    if output_dir:
        return Path(output_dir).expanduser().resolve()

    input_path = Path(input_dir).expanduser().resolve()
    root = Path(output_root).expanduser().resolve() if output_root else DEFAULT_OUTPUT_ROOT
    return root / f"{input_path.name}_lerobot"


# ─────────────────────────── Main conversion ────────────────────────────────

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
    default_gripper=1.0,
):
    output = Path(output_dir)
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    state_dim = len(arm_joint_names) + 1  # joints + gripper

    global_index = 0
    episode_records = []  # for episodes parquet
    all_states = []
    all_actions = []
    valid_episodes = []

    for ep_idx, bag_path in enumerate(bag_paths):
        print(f"\n[Episode {ep_idx}] Processing: {bag_path}")
        joint_states, camera_frames = read_rosbag(bag_path, joint_topic, camera_topics, typestore)
        if not joint_states:
            print(f"  SKIP: No joint states found in {bag_path}")
            continue

        timestamps, states, images = synchronize_data(
            joint_states,
            camera_frames,
            fps,
            arm_joint_names=arm_joint_names,
            gripper_joint_name=gripper_joint_name,
            default_gripper=default_gripper,
        )
        n_frames = len(timestamps)

        # Build action as the next recorded state.
        actions = np.roll(states, -1, axis=0)
        actions[-1] = states[-1]  # last action = last state

        chunk_idx = ep_idx // 1000
        chunk = f"chunk-{chunk_idx:03d}"
        file_idx = ep_idx % 1000  # within-chunk file index

        # ── Write videos ──────────────────────────────────────────────────
        video_frame_counts = {}
        for topic, cam_name in zip(camera_topics, camera_names):
            video_path = (
                output / "videos" / chunk
                / f"observation.images.{cam_name}"
                / f"episode_{ep_idx:06d}.mp4"
            )
            write_video(images[topic], video_path, fps)
            # Verify actual frame count from video file
            n_vid = get_video_frame_count(str(video_path))
            video_frame_counts[cam_name] = n_vid
            print(f"  [{cam_name}] video frames: {n_vid}")

        # ── Trim parquet rows to match shortest video ─────────────────────
        safe_n_frames = min(n_frames, *video_frame_counts.values())
        if safe_n_frames < n_frames:
            print(f"  Trimming parquet from {n_frames} → {safe_n_frames} frames to match video")
            n_frames = safe_n_frames
            timestamps = timestamps[:n_frames]
            states = states[:n_frames]
            actions = actions[:n_frames]

        # ── Write data parquet (NO image struct columns) ───────────────────
        parquet_dir = output / "data" / chunk
        parquet_dir.mkdir(parents=True, exist_ok=True)
        table = pa.table({
            "episode_index": pa.array(np.full(n_frames, ep_idx, dtype=np.int64)),
            "frame_index":   pa.array(np.arange(n_frames, dtype=np.int64)),
            "index":         pa.array(np.arange(global_index, global_index + n_frames, dtype=np.int64)),
            "timestamp":     pa.array(timestamps),
            "task_index":    pa.array(np.zeros(n_frames, dtype=np.int64)),
            "observation.state": pa.array(
                [states[i].tolist() for i in range(n_frames)],
                type=pa.list_(pa.float32()),
            ),
            "action": pa.array(
                [actions[i].tolist() for i in range(n_frames)],
                type=pa.list_(pa.float32()),
            ),
        })
        pq.write_table(table, parquet_dir / f"episode_{ep_idx:06d}.parquet")

        # ── Record episode metadata ────────────────────────────────────────
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
            # Use a slightly smaller timestamp to ensure we never hit the index=N boundary
            ep_record[f"videos/observation.images.{cam_name}/to_timestamp"] = (
                (video_frame_counts[cam_name] - 1.01) / fps
            )

        episode_records.append(ep_record)
        valid_episodes.append(ep_idx)
        all_states.append(states)
        all_actions.append(actions)
        global_index += n_frames

    total_episodes = len(valid_episodes)
    total_frames   = global_index
    state_names = [f"joint{i+1}" for i in range(len(arm_joint_names))] + ["gripper"]

    # ── meta/ directory ───────────────────────────────────────────────────
    meta_dir = output / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    # info.json — v3.0 path templates use chunk_index / file_index
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
        # v3.0 uses chunk_index / file_index (NOT episode_chunk / episode_index)
        "data_path":  "data/chunk-{chunk_index:03d}/episode_{file_index:06d}.parquet",
        "video_path": "videos/chunk-{chunk_index:03d}/{video_key}/episode_{file_index:06d}.mp4",
        "features": {
            # Standard index columns (required by LeRobotDataset.__getitem__)
            "episode_index": {"dtype": "int64",   "shape": [1], "names": None},
            "frame_index":   {"dtype": "int64",   "shape": [1], "names": None},
            "timestamp":     {"dtype": "float32", "shape": [1], "names": None},
            "index":         {"dtype": "int64",   "shape": [1], "names": None},
            "task_index":    {"dtype": "int64",   "shape": [1], "names": None},
            # Robot state & action
            "observation.state": {
                "dtype": "float32", "shape": [state_dim], "names": state_names,
            },
            "action": {
                "dtype": "float32", "shape": [state_dim], "names": state_names,
            },
            # Camera video features
            **{
                f"observation.images.{cam_name}": {
                    "dtype": "video",
                    "shape": [480, 640, 3],
                    "names": ["height", "width", "channels"],
                    "info": {
                        "video.fps":         float(fps),
                        "video.codec":       "mp4v",
                        "video.pix_fmt":     "yuv420p",
                        "video.is_depth_map": False,
                        "has_audio":         False,
                    },
                }
                for cam_name in camera_names
            },
        },
    }
    with open(meta_dir / "info.json", "w") as f:
        json.dump(info, f, indent=2)

    # tasks.parquet — must be indexed by 'task' (DataFrame index = task string)
    tasks_df = pd.DataFrame([{"task": task_description, "task_index": 0}]).set_index("task")
    pq.write_table(pa.Table.from_pandas(tasks_df), str(meta_dir / "tasks.parquet"))

    # episodes/chunk-000/episodes.parquet — nested directory, NOT meta/episodes.parquet
    episodes_df = pd.DataFrame(episode_records)
    ep_chunk_dir = meta_dir / "episodes" / "chunk-000"
    ep_chunk_dir.mkdir(parents=True, exist_ok=True)
    pq.write_table(
        pa.Table.from_pandas(episodes_df, preserve_index=False),
        str(ep_chunk_dir / "episodes.parquet"),
    )

    # stats.json — LeRobot loads meta/stats.json, not safetensors
    all_states_arr  = np.vstack(all_states)
    all_actions_arr = np.vstack(all_actions)
    stats = {}
    for key, arr in [("observation.state", all_states_arr), ("action", all_actions_arr)]:
        stats[key] = {
            "mean": arr.mean(axis=0).tolist(),
            "std":  arr.std(axis=0).tolist(),
            "min":  arr.min(axis=0).tolist(),
            "max":  arr.max(axis=0).tolist(),
        }
    # Camera entries need mean/std keys so --dataset.use_imagenet_stats can overwrite them.
    for cam_name in camera_names:
        stats[f"observation.images.{cam_name}"] = {
            "mean": [0.485, 0.456, 0.406],
            "std":  [0.229, 0.224, 0.225],
        }
    with open(meta_dir / "stats.json", "w") as f:
        json.dump(stats, f, indent=2)

    print(f"\n✓ Dataset written to {output}")
    print(f"  Episodes: {total_episodes}  |  Frames: {total_frames}")


# ─────────────────────────── Verification ───────────────────────────────────

def verify_dataset(dataset_path: str, tolerance_s: float = 0.05):
    """Quick sanity-check: load the dataset through LeRobot and read a few items."""
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError:
        print("lerobot not installed — skipping Python-level verification.")
        return

    print(f"\n[Verify] Loading dataset from {dataset_path} ...")
    ds = LeRobotDataset("verify_ds", root=dataset_path, tolerance_s=tolerance_s)
    n = len(ds)
    print(f"  Total frames: {n}")
    for idx in [0, n // 2, n - 1]:
        item = ds[idx]
        print(f"  [{idx}] task='{item['task']}'  timestamp={item['timestamp'].item():.3f}  "
              f"state_shape={item['observation.state'].shape}  "
              f"cam_shape={item['observation.images.' + list(ds.meta.camera_keys)[0].split('.')[-1]].shape}")
    print("  ✓ Verification passed")


# ─────────────────────────── CLI ────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Convert ROS2 rosbags → LeRobot v3.0 dataset")
    parser.add_argument(
        "--dir",
        default=str(DEFAULT_BAGS_DIR),
        help=(
            "Directory containing rosbag folders. "
            f"Default: {DEFAULT_BAGS_DIR}"
        ),
    )
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
        help=(
            "Root directory where converted LeRobot datasets are stored. "
            "When --output is not provided, the dataset is written to "
            "<output-root>/<rosbag-dir-name>_lerobot."
        ),
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Exact output dataset directory. Overrides --output-root when set.",
    )
    parser.add_argument("--fps",     type=int, default=30)
    parser.add_argument("--task",    default="pick up the cube", help="Language task description")
    parser.add_argument("--joint-topic",    default="/joint_states")
    parser.add_argument(
        "--arm-joint-names",
        nargs="+",
        default=[
            "xarm5_joint1",
            "xarm5_joint2",
            "xarm5_joint3",
            "xarm5_joint4",
            "xarm5_joint5",
        ],
        help="Arm joint names to extract from JointState by name, in model state/action order.",
    )
    parser.add_argument(
        "--gripper-joint-name",
        default="xarm_gripper_right_drive_joint",
        help="Gripper joint name to extract from JointState by name.",
    )
    parser.add_argument(
        "--default-gripper",
        type=float,
        default=1.0,
        help="Fallback gripper value if the configured gripper joint is absent in JointState.",
    )
    parser.add_argument("--camera-topics",  nargs="+",
                        default=["/camera1/realsense_camera/color/image_raw/compressed",
                                 "/camera2/realsense_camera/color/image_raw/compressed"])
    parser.add_argument("--camera-names",   nargs="+", default=["camera1", "camera2"])
    parser.add_argument(
        "--tolerance-s",
        type=float,
        default=0.05,
        help="Timestamp tolerance used when verifying dataset video/frame alignment.",
    )
    parser.add_argument("--verify",  action="store_true",
                        help="After conversion, verify dataset loads correctly via LeRobot")
    args = parser.parse_args()

    input_dir = Path(args.dir).expanduser().resolve()
    if not input_dir.exists():
        print(f"Input rosbag directory does not exist: {input_dir}")
        return

    if len(args.camera_topics) != len(args.camera_names):
        print("--camera-topics and --camera-names must have the same length.")
        return

    output_dir = resolve_output_dir(args.dir, args.output_root, args.output)
    output_dir.parent.mkdir(parents=True, exist_ok=True)

    bag_paths = sorted(
        p for p in input_dir.iterdir()
        if p.is_dir() and (p / "metadata.yaml").exists()
    )
    if not bag_paths:
        print(f"No rosbag directories found in {input_dir}")
        return

    print(f"Found {len(bag_paths)} rosbag(s) in {input_dir}")
    print(f"Writing LeRobot dataset to {output_dir}")
    convert_bags(
        bag_paths=bag_paths,
        output_dir=output_dir,
        fps=args.fps,
        task_description=args.task,
        joint_topic=args.joint_topic,
        camera_topics=args.camera_topics,
        camera_names=args.camera_names,
        arm_joint_names=args.arm_joint_names,
        gripper_joint_name=args.gripper_joint_name,
        default_gripper=args.default_gripper,
    )

    if args.verify:
        verify_dataset(str(output_dir), tolerance_s=args.tolerance_s)


if __name__ == "__main__":
    main()
