#!/usr/bin/env python3
"""Minimal fake Elite RTSI server (port 30004) for testing station_bridge without a robot.

Implements the handshake the SDK's RtsiClientInterface uses (V, v, O, S) and then streams
DATA packages (U) at the requested frequency. Joints move sinusoidally; robot mode cycles
POWER_OFF -> IDLE -> RUNNING so mode-change publishing can be observed.

    python3 fake_rtsi_server.py [--port 30004]
"""
import argparse
import math
import socket
import struct
import threading
import time

TYPES = {
    "timestamp": "DOUBLE",
    "speed_scaling": "DOUBLE",
    "robot_mode": "INT32",
    "safety_status": "INT32",
}
for n in ("actual_joint_positions", "actual_joint_speeds", "actual_joint_torques",
          "target_joint_positions", "actual_TCP_pose", "actual_TCP_speed", "actual_TCP_force"):
    TYPES[n] = "VECTOR6D"


def pkt(kind: str, payload: bytes = b"") -> bytes:
    return struct.pack(">HB", 3 + len(payload), ord(kind)) + payload


def recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("closed")
        buf += chunk
    return buf


def value(name, t, t0):
    w = 0.5
    q = [0.5 * math.sin(w * t + i) for i in range(6)]
    if name == "timestamp":
        return t - t0
    if name == "actual_joint_positions":
        return q
    if name == "target_joint_positions":
        return [x + 0.001 for x in q]
    if name == "actual_joint_speeds":
        return [0.5 * w * math.cos(w * t + i) for i in range(6)]
    if name == "actual_joint_torques":
        return [float(i) for i in range(6)]
    if name == "actual_TCP_pose":
        return [0.4, 0.1 * math.sin(t), 0.3, 0.0, 0.0, math.pi / 2]   # 90 deg about z
    if name == "actual_TCP_speed":
        return [0.0, 0.1 * math.cos(t), 0.0, 0.0, 0.0, 0.0]
    if name == "actual_TCP_force":
        return [0.0, 0.0, -9.81, 0.0, 0.0, 0.0]
    if name == "robot_mode":
        return [3, 5, 7][int((t - t0) / 3) % 3]
    if name == "safety_status":
        return 1
    if name == "speed_scaling":
        return 1.0
    raise KeyError(name)


def pack_value(name, v):
    kind = TYPES[name]
    if kind == "DOUBLE":
        return struct.pack(">d", v)
    if kind == "INT32":
        return struct.pack(">i", v)
    return struct.pack(">6d", *v)


def serve(conn, addr):
    print(f"client {addr}")
    recipe, freq, streaming = [], 250.0, False
    conn.settimeout(0.0005)
    t0 = time.time()
    next_send = time.time()
    try:
        while True:
            try:
                hdr = recv_exact(conn, 3)
                conn.settimeout(None)
                size, kind = struct.unpack(">HB", hdr)
                body = recv_exact(conn, size - 3)
                kind = chr(kind)
                if kind == "V":
                    conn.sendall(pkt("V", b"\x01"))
                elif kind == "v":
                    conn.sendall(pkt("v", struct.pack(">4I", 2, 14, 5, 123)))
                elif kind == "O":
                    freq = struct.unpack(">d", body[:8])[0]
                    recipe = body[8:].decode().split(",")
                    types = ",".join(TYPES.get(n, "NOT_FOUND") for n in recipe)
                    conn.sendall(pkt("O", b"\x01" + types.encode()))
                    print(f"recipe @ {freq} Hz: {recipe}")
                elif kind == "S":
                    streaming = True
                    conn.sendall(pkt("S", b"\x01"))
                elif kind == "P":
                    streaming = False
                    conn.sendall(pkt("P", b"\x01"))
                conn.settimeout(0.0005)
            except socket.timeout:
                pass
            if streaming and time.time() >= next_send:
                t = time.time()
                data = b"\x01" + b"".join(pack_value(n, value(n, t, t0)) for n in recipe)
                conn.sendall(pkt("U", data))
                next_send += 1.0 / freq
                if next_send < t:            # fell behind: don't burst
                    next_send = t + 1.0 / freq
    except (ConnectionError, OSError) as e:
        print(f"client {addr} gone: {e}")
    finally:
        conn.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=30004)
    args = ap.parse_args()
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen()
    print(f"fake RTSI on :{args.port}")
    while True:
        conn, addr = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        threading.Thread(target=serve, args=(conn, addr), daemon=True).start()


if __name__ == "__main__":
    main()
