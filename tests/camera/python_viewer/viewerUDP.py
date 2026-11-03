#!/usr/bin/env python3
"""Client for the ESP32-CAM server: commands over TCP, frames over UDP.

Usage:
    python cam_client_udp.py 192.168.4.1 3333

Keys (in the image window):
    p      request one image (snapshot), shown and saved to disk
    s      start the stream
    x      stop the stream
    q/Esc  quit

Protocol
    TCP (commands, PC -> ESP32): 4-byte little-endian length + "stream" | "stop" | "snapshot"
    TCP (ESP32 -> PC): only an error text (same framing), after which the server disconnects
    UDP (frames, ESP32 -> PC port 3334): packets of
        uint32 frame_id, uint16 packet_index, uint16 packet_count, uint32 frame_size  (little-endian)
        followed by up to 1400 bytes of JPEG data
"""
import argparse
import socket
import struct
import threading
import time

LEN = struct.Struct("<I")
HDR = struct.Struct("<IHHI")       # frame_id, packet_index, packet_count, frame_size  (12 bytes)
UDP_PORT = 3334
UDP_PAYLOAD = 1400
JPEG_MAGIC = b"\xff\xd8"


def send_command(sock, cmd: str):
    payload = cmd.encode("ascii")
    sock.sendall(LEN.pack(len(payload)) + payload)


class FrameAssembler:
    """Rebuilds frames from UDP packets. Incomplete frames are dropped, never waited for."""

    def __init__(self):
        self.frames = {}               # frame_id -> [bytearray, set(received indices), count]
        self.last_complete = -1
        self.completed = 0
        self.dropped = 0

    def add(self, packet: bytes):
        """Returns the JPEG bytes when a frame becomes complete, else None."""
        if len(packet) < HDR.size:
            return None
        fid, idx, count, size = HDR.unpack_from(packet)
        if fid <= self.last_complete or count == 0 or idx >= count:
            return None                # late packet of an old frame, or garbage

        entry = self.frames.get(fid)
        if entry is None:
            entry = self.frames[fid] = [bytearray(size), set(), count]
        buf, got, _ = entry
        data = packet[HDR.size:]
        start = idx * UDP_PAYLOAD
        buf[start:start + len(data)] = data
        got.add(idx)

        if len(got) < count:
            return None

        # Complete: anything older that is still incomplete will never be shown -> drop it
        for old in [k for k in self.frames if k <= fid]:
            if old != fid:
                self.dropped += 1
            del self.frames[old]
        self.last_complete = fid
        self.completed += 1
        return bytes(buf)


class UdpReceiver(threading.Thread):
    """Receives UDP packets, reassembles frames, keeps only the newest one."""

    def __init__(self, port=UDP_PORT, decode=True):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.decode = decode
        self.assembler = FrameAssembler()
        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.latest_frame = None
        self.frame_id = 0
        self.running = True
        self._win_t, self._win_n, self.fps = time.time(), 0, 0.0

    def run(self):
        if self.decode:
            import cv2
            import numpy as np
        while self.running:
            try:
                packet, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            jpeg = self.assembler.add(packet)
            if jpeg is None or not jpeg.startswith(JPEG_MAGIC):
                continue
            frame = None
            if self.decode:
                frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            now = time.time()
            with self.lock:
                self.latest_jpeg, self.latest_frame = jpeg, frame
                self.frame_id += 1
                self._win_n += 1
                if now - self._win_t >= 2.0:        # fps averaged over 2 s windows
                    self.fps = self._win_n / (now - self._win_t)
                    self._win_t, self._win_n = now, 0

    def stop(self):
        self.running = False
        self.sock.close()


class TcpErrorWatcher(threading.Thread):
    """The TCP connection only carries error messages back; report them and disconnects."""

    def __init__(self, sock):
        super().__init__(daemon=True)
        self.sock, self.error = sock, None

    def run(self):
        try:
            while True:
                head = self._recv_exact(LEN.size)
                (n,) = LEN.unpack(head)
                self.error = "server: " + self._recv_exact(n).decode("utf-8", "replace")
        except (ConnectionError, OSError) as e:
            if self.error is None:
                self.error = f"connection: {e}"

    def _recv_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("server closed the connection")
            buf += chunk
        return bytes(buf)


def main():
    import cv2
    import numpy as np

    ap = argparse.ArgumentParser(description="ESP32-CAM UDP client")
    ap.add_argument("ip", help="ESP32 address, e.g. 192.168.4.1")
    ap.add_argument("port", type=int, help="TCP command port, e.g. 3333")
    ap.add_argument("--udp-port", type=int, default=UDP_PORT, help="local UDP port for frames")
    args = ap.parse_args()

    rx = UdpReceiver(args.udp_port)           # bind UDP first, so no early frame is missed
    rx.start()

    sock = socket.create_connection((args.ip, args.port), timeout=5)
    sock.settimeout(None)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    watcher = TcpErrorWatcher(sock)
    watcher.start()
    print(f"connected to {args.ip}:{args.port}, frames on UDP {args.udp_port}")
    print("keys:  p = snapshot   s = stream   x = stop   q = quit")

    window = "ESP32-CAM (UDP)"
    cv2.namedWindow(window)
    img = np.zeros((480, 640, 3), np.uint8)
    shown_id, streaming, pending_snapshot, last_error = 0, False, False, None

    while True:
        with rx.lock:
            jpeg, frame, fid, fps = rx.latest_jpeg, rx.latest_frame, rx.frame_id, rx.fps
        a = rx.assembler

        if fid != shown_id and frame is not None:
            img, shown_id = frame, fid
            if pending_snapshot:
                name = time.strftime("snapshot_%Y%m%d_%H%M%S.jpg")
                with open(name, "wb") as f:
                    f.write(jpeg)
                print("saved", name)
                pending_snapshot = False

        err = watcher.error
        if err and err != last_error:
            print(err)
            last_error = err

        view = img.copy()
        status = "DISCONNECTED" if err else ("STREAMING" if streaming else "idle")
        text = f"{status}  {fps:4.1f} fps  dropped {a.dropped}/{a.completed + a.dropped}"
        cv2.putText(view, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow(window, view)

        key = cv2.waitKey(1) & 0xFF
        try:
            if key == ord("p"):
                send_command(sock, "snapshot")
                streaming, pending_snapshot = False, True
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
            break

    try:
        send_command(sock, "stop")
    except OSError:
        pass
    sock.close()
    rx.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
