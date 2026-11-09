# robotsimulator

MuJoCo tools for the Elite **EC66**, in C++ with CMake:

- **`simulator`** pretends to be the robot. A MuJoCo arm sits behind the controller's
  own network interface: JSON-RPC on **8055** and the 1024-byte monitor stream on **8056**
  every 8 ms. `ec_robot`, the bridge and the digital twin connect to it unchanged.
  An ImGui teach pendant switches teach/remote, presses the e-stop and releases the brakes.
- **`digitaltwin`** follows the real robot. It reads the monitor stream (8056) and puts
  the reported joint angles into the model, so the window shows what the controller
  says the arm is doing. An ImGui panel can also send commands over 8055 (see below).

```
simulator  (fake robot)  <--- 8055 / 8056 --->  ec_robot clients, bridge, digitaltwin
real EC66  192.168.0.11  <--- 8055 / 8056 --->  same clients
```

## Build

```bash
sudo apt install build-essential cmake git libglfw3-dev libasio-dev nlohmann-json3-dev libegl-dev libgl-dev
cmake -S . -B build
cmake --build build -j
```

CMake downloads and builds MuJoCo **4.0.0** and Dear ImGui **v1.91.9**. The first build
takes a few minutes. If you already have a MuJoCo checkout, add
`-DFETCHCONTENT_SOURCE_DIR_MUJOCO=/path/to/mujoco` to skip the download.

## Run

```bash
./build/simulator/simulator                       # fake robot on 127.0.0.1:8055/8056
./build/simulator/simulator --cold                # boots like the real one: teach mode, brakes closed
./build/simulator/simulator --bind 0.0.0.0        # reachable from other machines
./build/simulator/simulator --headless            # no window

./build/digitaltwin/digitaltwin                   # mirror the robot at 192.168.0.11
./build/digitaltwin/digitaltwin --ip 127.0.0.1    # mirror (and command) the simulator
./build/digitaltwin/digitaltwin --max-change 15   # tighter limit per "move joint" (default 30 deg)
```

Both programs find `library/models/scene.xml` through a compiled-in path, so you can run
them from any folder. To load a different model, pass its path as an argument.

Mouse: left drag rotates, right drag moves, shift for the horizontal variant, wheel zooms.
Keys: `c` cycles cameras, `esc` quits, `space` sends STOP in the twin.

## Digital twin commands

Mirroring uses only port 8056, which serves several clients, so the twin can run next to
the bridge or a script. Commands need port 8055, and **the controller accepts only one 8055
client**. While the twin is connected there, the bridge and your scripts are locked out.

There are two locks before anything moves:

1. **connect 8055** (button)
2. **arm commands** (checkbox). It switches itself off when the robot leaves remote mode,
   when the monitor goes stale (> 200 ms), or when 8055 closes.

**STOP** (button or space bar) works whenever 8055 is connected, armed or not. The joint
target sliders work while disarmed. The green ghost shows where the arm would go.
`move joint` is refused when any joint would change by more than `--max-change` degrees
(default 30). In testing, one careless slider drag asked for 294°. `power on` sets the tool you choose first
(default 1, the lab's 0.3 kg tool), then clears alarms, syncs and enables the servos.

The observed panel also checks the model: it compares the MuJoCo flange (site `tcp`) at
the reported joint angles with the flange position the controller reports. A few mm or
more means the model's kinematics or zero offsets don't match the arm.

## Layout

```
CMakeLists.txt                 dependencies (MuJoCo, ImGui, Asio, nlohmann_json, GLFW)
library/
  common.*                     model loading, home keyframe, helpers
  viewer.*                     GLFW window + MuJoCo scene + mouse camera + ImGui (both apps)
  egl_context.*                headless OpenGL (target ec_egl; not linked by the apps, see below)
  emulator/                    the fake controller (target ec_emulator)
    controller.*                 state machine + JSON-RPC commands, drives the MuJoCo servos
    motion.*                     joint/line moves, trajectory playback, streaming buffer
    kinematics.*                 FK / IK on the MuJoCo model (positiveKinematic / inverseKinematic)
    packet.*                     8056 monitor packet encoder
    servers.*                    8055 and 8056 TCP servers
  ec_robot/                    the C++ robot client, same code as ec_robot_cpp (target ec_robot)
  models/scene.xml             floor, lights, cameras, position servos, "home" keyframe
  models/ec66/                 the arm (converted from the elite-robots2 URDF, MIT meshes)
simulator/simulator.cc         fake robot: servers + 8 ms control timer + pendant GUI
digitaltwin/digitaltwin.cc     mirror of the real robot + command GUI
```

Each program runs a single `io_context`: a timer that re-arms itself (`expires_at(expiry() + period)`),
and a `while (!should_close) context.run_one();` loop. The simulator's control timer runs
every 8 ms. Physics then catches up to wall-clock time inside `Controller::tick`, so
simulated time follows real time even when a frame takes long. The twin runs blocking
client calls (connect, power on, moves) on worker threads so the window keeps drawing.

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
