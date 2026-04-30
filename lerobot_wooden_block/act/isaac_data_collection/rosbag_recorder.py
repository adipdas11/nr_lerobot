"""Start / stop rosbag recording via subprocess."""

from __future__ import annotations

import signal
import subprocess
import time
from pathlib import Path

from .config import CAMERA1_TOPIC, CAMERA2_TOPIC, JOINT_STATE_TOPIC


RECORD_TOPICS = [JOINT_STATE_TOPIC, CAMERA1_TOPIC, CAMERA2_TOPIC]


class RosbagRecorder:
    def __init__(self, base_dir: str | Path = "rosbags"):
        self._base_dir = Path(base_dir)
        self._proc: subprocess.Popen | None = None
        self._current_bag: Path | None = None

    def start(self, episode_id: int) -> Path:
        """Start recording; returns the bag directory path."""
        if self._proc is not None:
            raise RuntimeError("Recording already in progress")

        bag_dir = self._base_dir / f"episode_{episode_id:04d}"
        bag_dir.parent.mkdir(parents=True, exist_ok=True)

        cmd = [
            "ros2", "bag", "record",
            "--output", str(bag_dir),
            "--compression-mode", "none",
            *RECORD_TOPICS,
        ]
        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self._current_bag = bag_dir
        # Give ros2 bag a moment to start subscribing before the robot moves
        time.sleep(0.5)
        return bag_dir

    def stop(self) -> Path | None:
        """Stop recording and return the bag path."""
        if self._proc is None:
            return None
        self._proc.send_signal(signal.SIGINT)
        try:
            self._proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            self._proc.kill()
            self._proc.wait()
        self._proc = None
        bag = self._current_bag
        self._current_bag = None
        return bag

    def is_recording(self) -> bool:
        return self._proc is not None
