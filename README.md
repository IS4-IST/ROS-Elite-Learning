# robotsimulator

MuJoCo tools for the Elite **EC66**, in C++ with plain CMake:

- **`simulator`** pretends to be the robot. A MuJoCo arm sits behind the controller's
  own network interface: JSON-RPC on **8055** and the 1024-byte monitor stream on **8056**
  every 8 ms. Every client connects to it exactly as to the real arm. An ImGui teach
  pendant switches teach/remote, presses the e-stop and releases the brakes.
- **`digitaltwin`** follows the robot and is **the only program that commands it**. It
  reads the monitor stream (8056) into the model, so the window shows what the controller
  says the arm is doing. Commands come from its GUI (the operator) or, when built with
  ROS 2, from ROS clients. The operator decides whether ROS clients may command at all.
- **`planner`** finds collision-free joint motions with OMPL, using MuJoCo's collision
  checking in the same scene file. With ROS 2 it reads `/joint_states` and sends the motion
  to the twin as an action goal. It never talks to the robot itself.

```
                      8055 / 8056                     ROS 2 (optional)
simulator or EC66  <-------------->  digitaltwin  <-------------------->  planner, policies, ...
                                     (GUI + core)    /joint_states /robot_status
                                                      /stop /power /clear_alarm
                                                      /follow_joint_trajectory
```

## Build

```bash
sudo apt install build-essential cmake git libglfw3-dev libasio-dev nlohmann-json3-dev \
                 libegl-dev libgl-dev libompl-dev \
                 libavcodec-dev libswscale-dev libavutil-dev pkg-config
sudo apt install ros-humble-control-msgs     # only for the ROS interface; the rest is in ros-base
cmake -S . -B build
cmake --build build -j
```

CMake downloads and builds MuJoCo **4.0.0** and Dear ImGui **v1.91.9**. The first build
takes a few minutes. If you already have a MuJoCo checkout, add
`-DFETCHCONTENT_SOURCE_DIR_MUJOCO=/path/to/mujoco` to skip the download.

### ROS 2 is optional, and needs no sourcing

CMake looks for ROS 2 at `/opt/ros/$ROS_DISTRO`, else `/opt/ros/humble`, else
`/opt/ros/jazzy`. Override with `-DEC_ROS_DIR=/opt/ros/humble`, or switch it off with
`-DEC_WITH_ROS=OFF`. The configure step prints which one it uses, or why it skipped ROS.

- **Building:** you don't run `source setup.bash`. CMake only needs the install's prefix,
  and it sets `AMENT_PREFIX_PATH` for its own run (ROS's CMake scripts need it).
- **Running:** you don't source anything either. The twin and planner set `AMENT_PREFIX_PATH`
  and `LD_LIBRARY_PATH` themselves and restart once (`library/ros_env.h`), because the
  loader only reads `LD_LIBRARY_PATH` at process start. If ROS is already sourced, they
  don't restart.
- **What's ROS-aware:** only `digitaltwin/ros_interface.cc` and the planner include ROS
  headers. The simulator, the twin core and the libraries don't know ROS exists. Without
  ROS, the twin is GUI-only and the planner plans offline (`--start`).
- **Distro:** every ROS program that talks to the twin must use the same distro as the twin
  (Humble with Humble).

## Run

```bash
./build/simulator/simulator                       # fake robot on 127.0.0.1:8055/8056
./build/simulator/simulator --cold                # boots like the real one: teach mode, brakes closed
./build/simulator/simulator --bind 0.0.0.0        # reachable from other machines
./build/simulator/simulator --headless            # no window

./build/digitaltwin/digitaltwin                   # follow (and command) the robot at 192.168.0.11
./build/digitaltwin/digitaltwin --ip 127.0.0.1    # the simulator instead
#   --max-change 30   largest change of one "move joint" (deg)
#   --margin 10       clearance kept by planning and checked on every motion (mm)
#   --no-ros          don't start the ROS interface even if it was built

./build/planner/planner --goal -90 -90 90 -90 90 0              # plan from /joint_states, print
./build/planner/planner --goal -90 -90 90 -90 90 0 --execute    # ...and send it to the twin
./build/planner/planner --start home --goal ...                 # offline, no twin needed
```

**Give all three the same scene file.** The simulator and twin take it as the first
argument, the planner as `--model`. For example, with `library/models/scene_pillar.xml`
(the scene plus an obstacle):

```bash
./build/simulator/simulator library/models/scene_pillar.xml
./build/digitaltwin/digitaltwin library/models/scene_pillar.xml --ip 127.0.0.1
./build/planner/planner --model library/models/scene_pillar.xml --goal -90 -90 90 -90 90 0 --execute
```

Mouse: left drag rotates, right drag moves, shift for the horizontal variant, wheel zooms.
Keys: `c` cycles cameras, `esc` (or Ctrl+C in the terminal) quits, `space` sends STOP in the twin.

## Cameras (simulator)

The simulator can play your cameras too: it renders a model camera offscreen, encodes it
as H.264 and serves it on a TCP port, the way the ESP32-P4 nodes will.

```bash
./build/simulator/simulator library/models/scene_pillar.xml --camera scene_cam --camera wrist_cam:5001
nc 127.0.0.1 5000 | ffplay -f h264 -fflags nobuffer -           # watch it
./build/library/camera_probe 127.0.0.1 5000                    # fps, keyframes, kbit/s, latency
```

Options: `--camera NAME[:PORT]` (repeat; ports from 5000), `--camera-size 640x480`,
`--camera-fps 30`, `--camera-gop 15`, `--camera-bitrate 2000`, `--camera-codec libx264`.
Cameras need the window (they use its OpenGL context), so `--headless` has none. A camera
with no client isn't rendered or encoded.

The stream is plain H.264 Annex-B, nothing else on the wire, so any H.264 decoder plays
it. Each frame carries an SEI unit with its frame id and capture time (Unix ns); the
format is in `library/camera/stream_format.h`, and the P4 firmware should send the same.
Baseline profile, no B-frames, SPS/PPS on every keyframe. A new client gets a keyframe at
once; a client too slow for a frame skips to the next keyframe instead of queueing.

### Watching the cameras in the twin

```bash
./build/digitaltwin/digitaltwin library/models/scene_pillar.xml --ip 127.0.0.1 \
    --camera scene=127.0.0.1:5000 --camera wrist=127.0.0.1:5001
```

Each `--camera NAME=HOST:PORT` gets its own resizable panel: the live image (scaled to the
panel keeping its aspect ratio; untick "fit" for 1:1), and the stream's fps, kbit/s, decode
time, latency (arrival minus capture time, so clock offset included), frames skipped by the
camera and decode errors. A lost stream greys the last image out and reconnects every second.
The client decodes each frame as soon as its last byte arrives (the SEI says how many bytes
follow it), on the GUI thread, straight into an OpenGL texture.

## Who may command the robot (digital twin)

All commands, from the GUI or from ROS, go through one object, `twin::Core`
(`digitaltwin/twin_core.h`), and the same checks:

| | needs |
|---|---|
| STOP (button, space bar, `/stop`, cancelling a goal) | 8055 connected. Nothing else: always allowed. |
| operator: power, moves, plans (GUI) | **connect 8055** + **arm commands**. Locked while an external motion runs. |
| external: motions, `/power`, `/clear_alarm` (ROS) | the above + **accept external commands** (operator's checkbox) |

- **Arming drops by itself** when the robot leaves remote mode, the monitor is stale for
  more than 200 ms, or 8055 closes. The external grant drops with it.
- **Withdrawing a permission during a motion stops it.** Unticking "accept external
  commands" stops an external motion. Losing "arm commands" stops any motion.
- **The controller takes only one 8055 client.** While the twin holds it, nothing else
  connects. That's why everything else goes through the twin.

Other guards:
- **`move joint`** (one `moveByJoint`) is refused above `--max-change` (default 30°). In
  testing, one careless slider drag asked for 294°.
- **`power on`** sets the tool chosen in the GUI (default 1, the lab's 0.3 kg tool) and the
  speed override, both for the GUI and for `/power`.
- **Model check:** the observed panel compares the MuJoCo flange (site `tcp`) at the reported
  joints with the flange the controller reports. A few mm or more means the model's
  kinematics or zero offsets don't match the arm.

## Motions: what the twin checks before anything moves

Every motion is a joint trajectory, from the operator's "plan to target", a ROS path or a
ROS timed trajectory. Before sending, the twin:

1. **Times it.** A geometric path (waypoints only) gets straight segments in joint space
   with a trapezoidal speed profile at the **motion limits** (twin's Motion window, default
   30 deg/s and 60 deg/s²), stopping at each waypoint. A timed trajectory keeps its timing,
   but the twin samples it every 8 ms with straight lines and checks speed and acceleration
   against the same limits. Sparse timed points become velocity jumps and are refused.
2. **Checks collisions in its own scene.** Every 8 ms sample, and the straight motion between
   samples (at most 1° apart), must keep `--margin` from everything.
3. **Checks the start.** The arm must still be within 0.5° of the motion's first point.
4. **Runs it** through the controller's trajectory service. STOP during the upload cancels
   it, so it can't start afterwards. During execution, STOP sends `stop` and
   `stop_trajectory`. The motion counts as done when the controller reports stop and the
   arm is within 0.1° of the goal.

The operator's plans are previewed as a blue ghost, replayed in real time. The green ghost
is the joint-slider target.

## ROS 2 interface (twin, node `digitaltwin`)

| name | type | |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | every monitor packet (125 Hz); rad, rad/s, Nm; `joint1`..`joint6` |
| `/robot_status` | `diagnostic_msgs/DiagnosticStatus` | 10 Hz: mode, state, servo, synced, estop, collision, connected_8055, armed, external_control, active_motion, progress. Level OK / WARN (servos off, not remote) / ERROR (alarm, collision, e-stop, stale). |
| `/stop` | `std_srvs/Trigger` | always allowed |
| `/power` | `std_srvs/SetBool` | true = tool + power on + speed override, false = off; needs the grant |
| `/clear_alarm` | `std_srvs/Trigger` | needs the grant |
| `/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | motions, see below |

The camera publisher comes later.

**The action:**
- **Path or timed.** If every `time_from_start` is 0, it's a **path** and the twin times it.
  Otherwise it's a **timed** trajectory and the twin only checks the timing.
- **Start.** If the first point isn't the current joint state, the twin puts the current
  state in front (time 0).
- **Joint names.** `joint1`..`joint6` in any order, or empty for that order.
- **Refusals.** Every goal is accepted first, then checked. Refusals (no grant, robot
  moving, collision, limits, start mismatch) come back as an **aborted** result, with the
  reason in `error_string` and `error_code` = `INVALID_GOAL`. Failures during the motion use
  `PATH_TOLERANCE_VIOLATED`.
- **Feedback.** About 20 Hz: desired, actual, error.
- **Cancel.** Cancelling stops the arm. The result is then `canceled`.

```bash
ros2 topic echo /robot_status
ros2 service call /stop std_srvs/srv/Trigger
```

## Planner

```bash
./build/planner/planner --model library/models/scene_pillar.xml --goal -90 -90 90 -90 90 0 --execute
./build/planner/planner ... --execute --timed --speed 30 --acc 60   # send dense samples timed here
```

It prints whether the plain joint move would have been collision-free, the waypoints,
the timed trajectory and its closest approach to anything. With `--execute` it sends:
- **by default, the path** (its waypoints, times 0). The twin times it.
- **with `--timed`, dense 8 ms samples timed by the planner.** The twin refuses them if they
  exceed its motion limits.

It then prints the twin's feedback and the result. Ctrl+C cancels the goal, which stops the arm.

Options: `--margin` mm (default 10), `--speed`/`--acc` per joint (default 30 deg/s,
60 deg/s²), `--algorithm`, `--timeout`, `--attempts`, `--start` (offline).

How it plans (`library/planning/`):

- **Collisions:** MuJoCo on a private copy of the model. A pose is valid when every moving
  link stays at least `margin` from everything else, including the arm's own non-adjacent
  links. Meshes collide as their convex hulls (qhull), so concave parts count as solid.
- **Always-close pairs:** some pairs of the arm's own links are within a few cm in every
  pose (base/link2 at ~14 mm, wrist housings). The planner samples 500 random poses when it
  starts. Pairs that are close in all of them are only checked for penetration, and it
  lists them as `relaxed`.
- **Search:** OMPL RRTConnect in joint space. The straight move is tried first. Otherwise
  8 searches share the timeout and the path with the least joint travel wins, because single
  RRT paths often swing the wrist around for no reason.

**The planner and the twin only know what's in the model file:** the arm, the floor and
whatever you add. They don't see the table, the camera mount or people. Model the cell
before trusting either on the real arm.

## Tested on the simulator (pillar scene, ROS 2 Humble)

| test | result |
|---|---|
| goal without the operator's grant | aborted: "the operator has not granted external control" |
| path goal (twin times it) | 3 waypoints, arrived within 0.01°, feedback every 0.5 s |
| `--timed` at the twin's limits | 733 samples, done |
| `--timed` at 90 deg/s (twin allows 30) | refused: joint 1 above the acceleration limit |
| Ctrl+C on the planner mid-motion | canceled, arm stopped at joint 1 = −50° |
| operator unticks the grant mid-motion | aborted "stopped: the operator withdrew external control", arm stopped |
| operator STOP mid-motion | aborted "stopped by the operator", arm stopped |
| operator "plan to target" + execute | done |
| goal to where the arm already is | done: "already at the goal, nothing to do" |
| twin closed (SIGINT) mid-motion | twin stops the arm and exits; planner: "FAILED the twin's action server disappeared" (exit 4) |
| `/power`, `/stop` without 8055 | `success=false` with the reason |

Library tests (no ROS needed; start `build/simulator/simulator` first):
`build/library/test_rpc`, `test_monitor`, `test_robot`, `test_stream`. Turn them off with
`-DEC_BUILD_TESTS=OFF`.

## Layout

```
CMakeLists.txt                 dependencies (MuJoCo, ImGui, Asio, nlohmann_json, GLFW, OMPL, optional ROS 2)
library/
  common.*                     model loading, home keyframe, helpers
  viewer.*                     GLFW window + MuJoCo scene + mouse camera + ImGui
  ros_env.h                    run a ROS program without sourcing (sets env, restarts once)
  egl_context.*                headless OpenGL (target ec_egl; not linked by the apps, see below)
  emulator/                    the fake controller (target ec_emulator)
    controller.*                 state machine + JSON-RPC commands, drives the MuJoCo servos
    motion.*                     joint/line moves, trajectory playback, streaming buffer
    kinematics.*                 FK / IK on the MuJoCo model (positiveKinematic / inverseKinematic)
    packet.*                     8056 monitor packet encoder
    servers.*                    8055 and 8056 TCP servers
  ec_robot/                    async robot client, C++20 coroutines (target ec_robot)
    types.*                      Joints, Pose, Mode, State, Result<T>
    net.h                        connect with a timeout
    rpc_client.*                 8055 JSON-RPC: one reader, one writer queue, replies matched by id
    monitor.*                    8056 stream: latest status, wait_until(predicate, timeout)
    robot.*                      commands, moves, trajectory upload; errors as std::error_code
    streamer.*                   transparent transmission at 8 ms (tt_put), with safety aborts
    tests/                       test_rpc, test_monitor, test_robot, test_stream
  planning/                    OMPL + MuJoCo planner, timing, collision checks (target ec_planning)
  camera/                      H.264 camera streams (target ec_camera, FFmpeg)
    stream_format.*              the wire format: NAL splitting, timestamp SEI
    encoder.*                    RGB -> H.264 with libavcodec (libx264 by default)
    server.*                     TCP broadcast on your io_context, keyframe-aware
    decoder.*                    H.264 -> RGBA with libavcodec, low delay
    client.*                     async receiver: reconnects, cuts frames by the SEI, stats
    tools/camera_probe.cc        connect and report fps, keyframes, bitrate, latency
  models/scene.xml             floor, lights, cameras, position servos, "home" keyframe
  models/scene_pillar.xml      scene.xml + an obstacle, for trying the planner
  models/ec66/                 the arm (converted from the elite-robots2 URDF, MIT meshes)
simulator/simulator.cc         fake robot: servers + 8 ms control timer + pendant GUI
digitaltwin/
  twin_core.*                  the one place that commands the robot: permissions, checks, motions
  digitaltwin.cc               GUI front end (mirror, commands, motion panels)
  camera_view.*                a camera panel: client + OpenGL texture + numbers
  ros_interface.*              ROS 2 front end (only built with ROS)
planner/planner.cc             plan; with ROS read /joint_states and send to the twin
```

Threads:

- **Simulator.** One `io_context` on its own thread for the 8055/8056 servers and the 8 ms
  control timer. The rendering timer runs on the main thread, and so do the cameras (their
  own timer at the camera rate), because rendering needs the window's OpenGL context. Physics catches up to
  wall-clock time inside `Controller::tick`, so simulated time follows real time even when
  a frame takes long.
- **Twin.** Three threads, no blocking calls anywhere:
  - *main*: rendering timer and GUI, in the `while (!should_close) context.run_one();` loop.
    It reads a `View` snapshot of the core each frame.
  - *robot*: one `io_context` owns `ec::Robot` (RPC, monitor, motions) as coroutines. A
    STOP is just another coroutine on it and is answered while a trajectory uploads.
  - *planning*: OMPL, timing and collision checks. Motions hop there with
    `co_await co_spawn(planning_context, ..., use_awaitable)` and come back to the robot
    thread when done.

  With ROS, a 4-thread executor runs in separate callback groups, so a slow `/power` never
  delays `/stop`, the joint states or the action. Every core call takes a callback; nothing
  returns a future to wait on.

## The robot library (`ec_robot`) in short

- Everything is `asio::awaitable` on the caller's `io_context`. No locks except the
  monitor's latest-status copy, no threads of its own.
- Expected failures are `std::error_code` (categories `ec-controller`, `ec-monitor`,
  `ec-robot`, `ec-stream`), with `last_error_detail()` for the human-readable reason.
  Nothing throws for a refused command, a timeout or a lost connection.
- Builds with the Asio in Ubuntu 22.04 (1.18.1): no awaitable operators, no `as_tuple`;
  timeouts are timers that close the socket or cancel a wait.
- GCC 11–13 coroutine pitfalls avoided in this code: no braced init list inside a
  `co_await` expression (internal compiler error), no structured bindings on a
  `co_await` result, and `-Wno-mismatched-new-delete` for a false positive.

`ec_egl` is a separate target on purpose. MuJoCo picks its OpenGL loader from the
libraries already loaded in the process, so one program must link **either** EGL +
libOpenGL (headless) **or** a GLFW window, never both.

## What the simulator emulates

The emulated behaviour was checked against the lab controller (V3.11.2):

- Results come back as JSON text, so the version string arrives double-encoded.
- Encoder sync fails until the brakes are released (pendant panel). Releasing them also syncs.
- Remote-only commands are refused outside remote mode.
- `moveByLine` with `target_pose` is rejected. Use `targetPos`, as `ec_robot` does.
- moveByJoint/moveByLine report "stopped" only once the arm is in position.
- A second 8055 client is accepted and closed at once, like the real controller.
- Transparent transmission (streaming), the trajectory service, FK/IK, tools/payload,
  e-stop → alarm → clearAlarm, and collisions (robot contact → collision state).

It does **not** reproduce the real arm's dynamics. The servo gains in `scene.xml` were raised
(kp 20000/5000) so the arm settles within 0.01° in about 0.2 s. With the old gains the
in-position check timed out. Masses, friction and armature come from the URDF or are
guesses, so they aren't identified from the real arm. The 100 ms lookahead vibration seen
on the real robot doesn't show up here.

## Model notes (check before trusting the dynamics)

- **Masses** add up to ~10.5 kg (excluding the base), which looks light for an EC66.
- **Joint 2 range** is [-180, 0] deg in the URDF, which is probably tighter than the hardware.
  The twin shows an observed angle in red when it falls outside the model's range.
- Collision geometry uses the full visual meshes. That's fine for one arm. For contact-heavy
  scenes, replace it with simpler shapes.
