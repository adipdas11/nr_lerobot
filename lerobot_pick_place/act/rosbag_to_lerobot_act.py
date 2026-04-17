#!/usr/bin/env python3
"""
Convert ROS2 rosbags to LeRobot v3.0 dataset format for ACT training.

The format is identical to the SmolVLA converter — LeRobot uses the same
dataset schema for all policies (parquet + MP4 videos). The only difference
at training time is --policy.type act instead of smolvla.

Usage:
    python rosbag_to_lerobot_act.py \
        --dir act/rosbags \
        --output act/lerobot_converted_dataset/pick_place_act \
        --fps 30

Then train with:
    python -m lerobot.scripts.lerobot_train \
        --policy.type act \
        --dataset.repo_id my_act_dataset \
        --dataset.root act/lerobot_converted_dataset/pick_place_act \
        --dataset.use_imagenet_stats true \
        --wandb.enable false \
        --output_dir act/training_output \
        --policy.push_to_hub false \
        --batch_size 8 \
        --num_workers 4
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
    if n_frames < 1:
        n_frames = 1
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
    output_path.parent.mkdir(parents=True, exist_ok=True)
    h, w = frames[0].shape[:2]
    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h)
    )
    for frame in frames:
        writer.write(frame)
    writer.release()
    return len(frames)


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
    episode_records = []
    all_states = []
    all_actions = []
    valid_episodes = []

    for ep_idx, bag_path in enumerate(bag_paths):
        print(f"\n[Episode {ep_idx}] {bag_path}")
        joint_states, camera_frames = read_rosbag(bag_path, joint_topic, camera_topics, typestore)
        if not joint_states:
            print(f"  SKIP: no joint states in {bag_path}")
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

        # Action = next state (absolute position targets).
        actions = np.roll(states, -1, axis=0)
        actions[-1] = states[-1]

        chunk_idx = ep_idx // 1000
        chunk = f"chunk-{chunk_idx:03d}"
        file_idx = ep_idx % 1000

        # Write MP4 videos.
        video_frame_counts = {}
        for topic, cam_name in zip(camera_topics, camera_names):
            video_path = (
                output / "videos" / chunk
                / f"observation.images.{cam_name}"
                / f"episode_{ep_idx:06d}.mp4"
            )
            write_video(images[topic], video_path, fps)
            n_vid = get_video_frame_count(str(video_path))
            video_frame_counts[cam_name] = n_vid
            print(f"  [{cam_name}] {n_vid} frames")

        # Trim to shortest video.
        safe_n_frames = min(n_frames, *video_frame_counts.values())
        if safe_n_frames < n_frames:
            print(f"  Trimming {n_frames} → {safe_n_frames} frames")
            n_frames = safe_n_frames
            timestamps = timestamps[:n_frames]
            states = states[:n_frames]
            actions = actions[:n_frames]

        # Write parquet (no image columns — videos are separate).
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
        raise RuntimeError("No valid episodes were converted. Check rosbag contents.")

    total_episodes = len(valid_episodes)
    total_frames = global_index
    state_names = [f"joint{i+1}" for i in range(len(arm_joint_names))] + ["gripper"]

    # meta/info.json
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
            "observation.state": {
                "dtype": "float32", "shape": [state_dim], "names": state_names,
            },
            "action": {
                "dtype": "float32", "shape": [state_dim], "names": state_names,
            },
            **{
                f"observation.images.{cam_name}": {
                    "dtype": "video",
                    "shape": [480, 640, 3],
                    "names": ["height", "width", "channels"],
                    "info": {
                        "video.fps":          float(fps),
                        "video.codec":        "mp4v",
                        "video.pix_fmt":      "yuv420p",
                        "video.is_depth_map": False,
                        "has_audio":          False,
                    },
                }
                for cam_name in camera_names
            },
        },
    }
    with open(meta_dir / "info.json", "w") as f:
        json.dump(info, f, indent=2)

    # tasks.parquet
    tasks_df = pd.DataFrame([{"task": task_description, "task_index": 0}]).set_index("task")
    pq.write_table(pa.Table.from_pandas(tasks_df), str(meta_dir / "tasks.parquet"))

    # episodes parquet
    episodes_df = pd.DataFrame(episode_records)
    ep_chunk_dir = meta_dir / "episodes" / "chunk-000"
    ep_chunk_dir.mkdir(parents=True, exist_ok=True)
    pq.write_table(
        pa.Table.from_pandas(episodes_df, preserve_index=False),
        str(ep_chunk_dir / "episodes.parquet"),
    )

    # stats.json — ACT uses ImageNet stats for images (applied by training flag),
    # but state/action stats still needed for normalisation.
    all_states_arr = np.vstack(all_states)
    all_actions_arr = np.vstack(all_actions)
    stats = {}
    for key, arr in [("observation.state", all_states_arr), ("action", all_actions_arr)]:
        stats[key] = {
            "mean": arr.mean(axis=0).tolist(),
            "std":  arr.std(axis=0).tolist(),
            "min":  arr.min(axis=0).tolist(),
            "max":  arr.max(axis=0).tolist(),
        }
    for cam_name in camera_names:
        stats[f"observation.images.{cam_name}"] = {}
    with open(meta_dir / "stats.json", "w") as f:
        json.dump(stats, f, indent=2)

    print(f"\n✓ ACT dataset written to {output}")
    print(f"  Episodes : {total_episodes}")
    print(f"  Frames   : {total_frames}")
    print(f"  State dim: {state_dim}  ({state_names})")
    print(f"  Cameras  : {camera_names}")


# ─────────────────────────── Validation ─────────────────────────────────────

def validate_dataset(dataset_path: str):
    """Validate the converted dataset structure without loading through LeRobot."""
    p = Path(dataset_path)
    errors = []

    required = [
        p / "meta" / "info.json",
        p / "meta" / "stats.json",
        p / "meta" / "tasks.parquet",
        p / "meta" / "episodes" / "chunk-000" / "episodes.parquet",
    ]
    for req in required:
        if not req.exists():
            errors.append(f"Missing: {req}")

    data_files = list((p / "data").rglob("*.parquet")) if (p / "data").exists() else []
    video_files = list((p / "videos").rglob("*.mp4")) if (p / "videos").exists() else []

    if not data_files:
        errors.append("No parquet data files found under data/")
    if not video_files:
        errors.append("No MP4 video files found under videos/")

    if errors:
        print("\n[Validate] ERRORS:")
        for e in errors:
            print(f"  ✗ {e}")
        return False

    print(f"\n[Validate] ✓ Dataset structure OK")
    print(f"  Parquet files : {len(data_files)}")
    print(f"  Video files   : {len(video_files)}")

    with open(p / "meta" / "info.json") as f:
        info = json.load(f)
    print(f"  Episodes      : {info['total_episodes']}")
    print(f"  Frames        : {info['total_frames']}")
    print(f"  FPS           : {info['fps']}")
    return True


# ─────────────────────────── CLI ────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Convert ROS2 rosbags → LeRobot v3.0 dataset for ACT training"
    )
    parser.add_argument("--dir", default=str(DEFAULT_BAGS_DIR),
                        help=f"Directory of rosbag folders. Default: {DEFAULT_BAGS_DIR}")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT),
                        help="Root for output datasets (used when --output is omitted)")
    parser.add_argument("--output", default=None,
                        help="Exact output dataset path (overrides --output-root)")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--task", default="pick up the cube")
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument("--arm-joint-names", nargs="+",
                        default=["xarm5_joint1", "xarm5_joint2", "xarm5_joint3",
                                 "xarm5_joint4", "xarm5_joint5"])
    parser.add_argument("--gripper-joint-name",
                        default="xarm_gripper_right_drive_joint")
    parser.add_argument("--default-gripper", type=float, default=1.0)
    parser.add_argument("--camera-topics", nargs="+",
                        default=["/camera1/realsense_camera/color/image_raw/compressed",
                                 "/camera2/realsense_camera/color/image_raw/compressed"])
    parser.add_argument("--camera-names", nargs="+", default=["camera1", "camera2"])
    parser.add_argument("--validate", action="store_true",
                        help="Validate dataset structure after conversion")
    args = parser.parse_args()

    input_dir = Path(args.dir).expanduser().resolve()
    if not input_dir.exists():
        print(f"ERROR: Rosbag directory not found: {input_dir}")
        return 1

    if len(args.camera_topics) != len(args.camera_names):
        print("ERROR: --camera-topics and --camera-names must have the same length.")
        return 1

    if args.output:
        output_dir = Path(args.output).expanduser().resolve()
    else:
        root = Path(args.output_root).expanduser().resolve()
        output_dir = root / f"{input_dir.name}_act_lerobot"

    bag_paths = sorted(
        p for p in input_dir.iterdir()
        if p.is_dir() and (p / "metadata.yaml").exists()
    )
    if not bag_paths:
        print(f"ERROR: No rosbag directories found in {input_dir}")
        return 1

    print(f"Found {len(bag_paths)} rosbag(s)")
    print(f"Output: {output_dir}")

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

    if args.validate:
        validate_dataset(str(output_dir))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
