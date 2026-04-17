#!/usr/bin/env python3
"""IPC helpers for the split ACT runtime (Unix-domain socket, pickle framing)."""

from __future__ import annotations

import pickle
import socket
import struct
from typing import Any


def send_message(sock: socket.socket, payload: Any) -> None:
    data = pickle.dumps(payload, protocol=pickle.HIGHEST_PROTOCOL)
    sock.sendall(struct.pack("!Q", len(data)))
    sock.sendall(data)


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("Socket closed while receiving payload")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def recv_message(sock: socket.socket) -> Any:
    header = recv_exact(sock, 8)
    (size,) = struct.unpack("!Q", header)
    payload = recv_exact(sock, size)
    return pickle.loads(payload)
