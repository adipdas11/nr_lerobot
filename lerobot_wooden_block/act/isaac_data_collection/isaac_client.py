"""Lightweight TCP client that talks to the Isaac Sim MCP extension (port 8766).

The extension uses a simple JSON-over-TCP protocol identical to the one in
isaacsim-mcp-server/isaac_mcp/connection.py:
  send:  {"type": "command_type", "params": {...}}
  recv:  {"status": "success"|"error", "result": {...}, "message": "..."}
"""

from __future__ import annotations

import json
import socket
import time
from typing import Any


class IsaacClient:
    """Persistent TCP client to the Isaac Sim MCP socket server."""

    def __init__(self, host: str = "localhost", port: int = 8766, timeout: float = 30.0):
        self._host = host
        self._port = port
        self._timeout = timeout
        self._sock: socket.socket | None = None

    # ── connection lifecycle ───────────────────────────────────────────────────

    def connect(self, retries: int = 5, delay: float = 1.0) -> None:
        for attempt in range(retries):
            try:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._sock.settimeout(self._timeout)
                self._sock.connect((self._host, self._port))
                return
            except OSError as exc:
                self._sock = None
                if attempt < retries - 1:
                    time.sleep(delay)
                else:
                    raise ConnectionError(
                        f"Cannot reach Isaac Sim MCP at {self._host}:{self._port} "
                        f"after {retries} attempts: {exc}"
                    ) from exc

    def disconnect(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def __enter__(self) -> "IsaacClient":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()

    # ── low-level send / receive ───────────────────────────────────────────────

    def _send(self, command_type: str, params: dict[str, Any] | None = None) -> dict:
        if self._sock is None:
            self.connect()
        payload = json.dumps({"type": command_type, "params": params or {}}).encode()
        self._sock.sendall(payload)

        chunks: list[bytes] = []
        while True:
            chunk = self._sock.recv(65536)
            if not chunk:
                break
            chunks.append(chunk)
            data = b"".join(chunks)
            try:
                response = json.loads(data.decode())
                if response.get("status") == "error":
                    raise RuntimeError(f"Isaac error: {response.get('message', '?')}")
                return response.get("result", response)
            except json.JSONDecodeError:
                continue  # wait for more bytes

        raise RuntimeError("Isaac Sim closed connection before sending a complete response")

    # ── high-level helpers ────────────────────────────────────────────────────

    def ping(self) -> bool:
        try:
            self._send("scene.info")
            return True
        except Exception:
            return False

    def get_prim_info(self, prim_path: str) -> dict:
        return self._send("scene.get_prim_info", {"prim_path": prim_path})

    def transform_object(
        self,
        prim_path: str,
        position: list[float] | None = None,
        rotation_deg: list[float] | None = None,
    ) -> dict:
        """Move/rotate a prim. rotation_deg = [rx, ry, rz] in degrees (Euler XYZ)."""
        params: dict[str, Any] = {"prim_path": prim_path}
        if position is not None:
            params["position"] = position
        if rotation_deg is not None:
            params["rotation"] = rotation_deg
        return self._send("objects.transform", params)

    def execute_script(self, code: str) -> dict:
        """Run arbitrary Python inside the Isaac Sim process."""
        return self._send("simulation.execute_script", {"code": code})

    def get_block_pose(self, prim_path: str) -> tuple[list[float], float]:
        """Return (world_xyz, yaw_rad) for a block prim."""
        info = self.get_prim_info(prim_path)
        pos = info.get("transform", {}).get("position", [0.0, 0.0, 0.0])

        # Use execute_script to read the yaw (rotation around Z).
        script = f"""
import json, math
from pxr import UsdGeom, Gf
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("{prim_path}")
xf = UsdGeom.Xformable(prim)
mat = xf.ComputeLocalToWorldTransform(0)
rot = Gf.Matrix3d(mat).ExtractRotation()
angles = rot.Decompose(Gf.Vec3d(0,0,1), Gf.Vec3d(0,1,0), Gf.Vec3d(1,0,0))
print(json.dumps({{"yaw": math.radians(angles[0])}}))
"""
        try:
            result = self.execute_script(script)
            output = result.get("output", "")
            for line in output.splitlines():
                line = line.strip()
                if line.startswith("{") and "yaw" in line:
                    yaw = json.loads(line)["yaw"]
                    return pos, yaw
        except Exception:
            pass
        return pos, 0.0
