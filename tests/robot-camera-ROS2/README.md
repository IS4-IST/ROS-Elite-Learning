# station_bridge

One ROS 2 node that publishes the Elite CS66 state (RTSI, 250 Hz) and the ESP32-CAM stream.

## Build

```bash
sudo apt install libasio-dev                          # standalone asio for the camera receiver
# Elite CS SDK: ppa:elite-robots/cs-robot-series-sdk -> elite-cs-series-sdk
cd ~/ros2_ws/src && cp -r /path/to/station_bridge .
cd ~/ros2_ws && colcon build --packages-select station_bridge
source install/setup.bash
```

If the SDK is installed somewhere other than /usr, add `--cmake-args -DCMAKE_PREFIX_PATH=/path/to/sdk`.

## Run

```bash
ros2 launch station_bridge station.launch.py                       # uses config/station.yaml
ros2 run station_bridge station_bridge_node --ros-args -p robot_ip:=192.168.1.200 -p camera.enabled:=false
```

## Topics

| topic | type | QoS | content |
|---|---|---|---|
| `/joint_states` | sensor_msgs/JointState | reliable, depth 10 | actual q, qd, torque |
| `~/joint_targets` | sensor_msgs/JointState | reliable | target q |
| `~/tcp_pose` | geometry_msgs/PoseStamped | reliable | TCP in `base_frame` (rotation vector → quaternion) |
| `~/tcp_twist` | geometry_msgs/TwistStamped | reliable | TCP velocity |
| `~/tcp_wrench` | geometry_msgs/WrenchStamped | reliable | TCP force/torque |
| `~/speed_scaling` | std_msgs/Float64 | reliable | |
| `~/robot_mode`, `~/safety_mode` | std_msgs/Int32 | transient_local | published on change only |
| `/camera/image/compressed` | sensor_msgs/CompressedImage | best effort, keep last 1 | JPEG bytes as received, not decoded |

`~` = `/station_bridge/`. All stamps are the PC time at reception.

Viewing the camera: `ros2 run rqt_image_view rqt_image_view` (pick the compressed topic). If a node needs
a raw `sensor_msgs/Image`, use `image_transport republish` or decode in that node, so only consumers pay for decoding.

`joint_names` must match your URDF, otherwise `robot_state_publisher` won't use `/joint_states`.

## Test without hardware

```bash
python3 test/fake_rtsi_server.py          # fake controller on :30004
ros2 run station_bridge station_bridge_node --ros-args -p robot_ip:=127.0.0.1 -p camera.enabled:=false
ros2 topic hz /joint_states               # ~250
```
