# p4_h264_streamer

ESP32-P4 camera node (Waveshare ESP32-P4-ETH + OV5647 "RPi Camera (B)"). Serves the camera as
H.264 over TCP in the repo's camera format (`libraries/camera/stream_format.h`), so anything
that reads the simulator's cameras reads this one too:

```bash
nc <ip> 5000 | ffplay -f h264 -fflags nobuffer -
./build/library/camera_probe <ip> 5000
./build/digitaltwin/digitaltwin library/models/scene_pillar.xml --camera scene=<ip>:5000
```

```
OV5647 --MIPI-CSI--> ISP (RAW10 -> YUV420) --> hardware H.264 --> stamp SEI --> camera::Server --TCP--> clients
```

- 1280x960 (2x2 binning of the full sensor), up to 45 fps, baseline, SPS/PPS on every IDR.
- Every frame carries the stamp SEI: frame_id (+1 per camera frame, a gap = a frame not
  encoded), capture time (Unix ns, camera clock), and `follow` (bytes of the frame after it).
- The TCP side is the repo's own `camera::Server` (compiled from `libraries/camera`): many
  clients, each starts at a keyframe, a slow client skips to the next keyframe. A joining or
  lagging client makes the encoder produce an IDR at once.

## Build and flash (ESP-IDF >= 5.4, tested to build with 6.1)

```bash
cd tests/camera/p4_h264_streamer
idf.py set-target esp32p4
idf.py build flash monitor
```

Settings (`idf.py menuconfig` -> "P4 camera streamer"): port (5000), bitrate (4000 kbit/s),
keyframe interval (30), QP range, NTP server.

## Network and clock

The board takes its address by DHCP and logs it with the command to watch the stream. With
the cable straight into a PC, let NetworkManager run DHCP on that port:

```bash
nmcli connection add type ethernet ifname <eth> con-name p4cam ipv4.method shared
nmcli connection up p4cam          # the board gets 10.42.0.x
```

`capture_ns` is the board's clock, set by SNTP: the DHCP server's NTP option if it sends one,
else the configured server (default pool.ntp.org, which needs internet). On an isolated lab
network, run an NTP server on the PC (e.g. chrony with `allow 10.42.0.0/24` and
`local stratum 10`) and set its address in menuconfig. Until the clock is set the log says
"clock NOT synchronised" and the twin's latency figure is meaningless (the stream still works).

## Log

```
p4cam: 45.0 fps | 3980 kbit/s | encode 8.1 ms | keyframes 2 | skipped 0 | clients 1
```

`skipped`: camera frames that weren't encoded (the capture task fell behind).

## PC test (no board)

`host_test/` runs the firmware's stamping code and server against the twin's `camera::Client`
over localhost, with frames from a file, and checks every frame decodes in order:

```bash
ffmpeg -f lavfi -i testsrc2=size=1280x960:rate=30 -frames:v 120 -c:v libx264 -profile:v baseline \
       -preset ultrafast -tune zerolatency -g 30 -x264-params repeat-headers=1 -f h264 /tmp/test.h264
cmake -S host_test -B /tmp/p4test && cmake --build /tmp/p4test
/tmp/p4test/stream_check /tmp/test.h264      # ... -> PASS
```
