#!/usr/bin/env python3
"""Client for the ESP32-CAM Asio server.

Usage:
    python cam_client.py 192.168.4.1 3333

Keys (in the image window):
    p      request one image (snapshot), shown and saved to disk
    s      start the stream
    x      stop the stream
    q/Esc  quit

Protocol (both directions): 4-byte little-endian uint32 length, then that many bytes.
    PC -> ESP32:  "stream" | "stop" | "snapshot"
    ESP32 -> PC:  a JPEG frame, or an error text (after which the server disconnects)
"""
import argparse
import socket
import struct
import threading
import time

import cv2
import numpy as np

LEN = struct.Struct("<I")          # little-endian uint32, matching the ESP32's native order
JPEG_MAGIC = b"\xff\xd8"           # every JPEG starts with these two bytes


def send_command(sock, cmd: str):
    payload = cmd.encode("ascii")
    sock.sendall(LEN.pack(len(payload)) + payload)


def recv_exact(sock, n: int) -> bytes:
    """TCP delivers data in arbitrary pieces: keep reading until exactly n bytes arrived."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("server closed the connection")
        buf += chunk
    return bytes(buf)


def recv_message(sock) -> bytes:
    (length,) = LEN.unpack(recv_exact(sock, LEN.size))
    return recv_exact(sock, length)


class Receiver(threading.Thread):
    """Reads messages in the background so the window never blocks on the network."""

    def __init__(self, sock):
        super().__init__(daemon=True)
        self.sock = sock
        self.lock = threading.Lock()
        self.latest_jpeg = None        # newest frame, replaced as new ones arrive
        self.frame_id = 0              # increments on every frame
        self.error = None              # error text from the server, or connection error
        self.fps = 0.0

    def run(self):
        t_prev = time.time()
        try:
            while True:
                msg = recv_message(self.sock)
                if msg.startswith(JPEG_MAGIC):
                    now = time.time()
                    with self.lock:
                        self.latest_jpeg = msg
                        self.frame_id += 1
                        self.fps = 0.8 * self.fps + 0.2 / max(now - t_prev, 1e-6)
                    t_prev = now
                else:
                    with self.lock:
                        self.error = "server: " + msg.decode("utf-8", errors="replace")
        except (ConnectionError, OSError) as e:
            with self.lock:
                if self.error is None:
                    self.error = f"connection: {e}"


def main():
    ap = argparse.ArgumentParser(description="ESP32-CAM client")
    ap.add_argument("ip", help="ESP32 address, e.g. 192.168.4.1")
    ap.add_argument("port", type=int, help="server port, e.g. 3333")
    args = ap.parse_args()

    sock = socket.create_connection((args.ip, args.port), timeout=5)
    sock.settimeout(None)              # blocking reads in the receiver thread
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"connected to {args.ip}:{args.port}")
    print("keys:  p = snapshot   s = stream   x = stop   q = quit")

    rx = Receiver(sock)
    rx.start()

    window = "ESP32-CAM"
    cv2.namedWindow(window)
    img = np.zeros((480, 640, 3), np.uint8)
    shown_id = 0
    streaming = False
    pending_snapshot = False
    last_error = None

    while True:
        with rx.lock:
            jpeg, fid, fps, err = rx.latest_jpeg, rx.frame_id, rx.fps, rx.error

        if fid != shown_id and jpeg is not None:
            frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                img = frame
            shown_id = fid
            if pending_snapshot:
                name = time.strftime("snapshot_%Y%m%d_%H%M%S.jpg")
                with open(name, "wb") as f:
                    f.write(jpeg)
                print("saved", name)
                pending_snapshot = False

        if err and err != last_error:
            print(err)
            last_error = err

        view = img.copy()
        status = "STREAMING" if streaming else "idle"
        if err:
            status = "DISCONNECTED"
        cv2.putText(view, f"{status}  {fps:4.1f} fps" if streaming else status,
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow(window, view)

        key = cv2.waitKey(1) & 0xFF
        try:
            if key == ord("p"):
                send_command(sock, "snapshot")
                streaming = False          # the server switches to snapshot mode
                pending_snapshot = True
            elif key == ord("s"):
                send_command(sock, "stream")
                streaming = True
            elif key == ord("x"):
                send_command(sock, "stop")
                streaming = False
            elif key in (ord("q"), 27):
                break
        except OSError as e:
            print("send failed:", e)

        if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
            break                          # window closed with the mouse

    try:
        send_command(sock, "stop")
    except OSError:
        pass
    sock.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
