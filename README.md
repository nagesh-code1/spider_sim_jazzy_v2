# spider_sim v2 (fixed) — ROS 2 Jazzy + Gazebo Harmonic

A 4-legged spider robot with a 3-joint arm, simulated in Gazebo Sim (Harmonic)
with `gz_ros2_control`. One ament_python package.

- 4 legs, 12 position-controlled joints (coxa / femur / tibia per leg)
- 3-DOF arm (base yaw, shoulder, elbow) with inverse kinematics
- diagonal-pair trot gait driven by `/cmd_vel`
- gripper interface (software only, no physics grasp yet)
- pick-and-place mission state machine
- scripted 2-minute demo and headless video recording

All commands are also in [`docs/commands.pdf`](docs/commands.pdf).

## Requirements
Ubuntu 24.04 (or WSL2 Ubuntu 24.04) with ROS 2 Jazzy:
```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ffmpeg
```

## Build
```bash
mkdir -p ~/spider_v2_ws/src && cd ~/spider_v2_ws/src
git clone <this repo url> spider_sim
cd ~/spider_v2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select spider_sim
source install/setup.bash
```

## Run
```bash
ros2 launch spider_sim sim.launch.py                    # with the Gazebo window
ros2 launch spider_sim sim.launch.py headless:=true     # no window (lighter)
```
Check the controllers (all 3 should be `active`):
```bash
ros2 control list_controllers
```

## Drive it (new terminal, source install/setup.bash first)
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"     # walk forward
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.6}}"    # turn left
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"                    # stop
```

## Arm, gripper, mission
`/arm_target` is a `geometry_msgs/Point`: x = forward, y = base yaw (rad), z = up,
measured from the shoulder pivot.
```bash
ros2 topic pub --once /arm_target geometry_msgs/msg/Point "{x: 0.18, y: 0.0, z: 0.05}"
ros2 topic pub --once /gripper/command std_msgs/msg/Bool "{data: true}"    # grab
ros2 topic pub --once /gripper/command std_msgs/msg/Bool "{data: false}"   # release
ros2 topic pub --once /mission std_msgs/msg/String "{data: 'pick_and_place'}"
ros2 topic echo /mission/state
```

## Manual leg pose
Stop the robot first; the gait node goes quiet 1 s after it stops.
```bash
ros2 topic pub --once /leg_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.8, 1.1, 0.0, 0.8, 1.1, 0.0, 0.8, 1.1, 0.0, 0.8, 1.1]}"
```

## 2-minute demo + recording
```bash
ros2 launch spider_sim sim.launch.py headless:=true record:=true record_dir:=$HOME/spider_rec_v2
# second terminal, once the controllers are active:
ros2 run spider_sim demo --ros-args -p use_sim_time:=true
```
Frames land in `~/spider_rec_v2/overview` and `~/spider_rec_v2/chase`. Make an MP4:
```bash
cd ~/spider_rec_v2
mkdir -p seq && i=0; for f in $(ls overview | sort -V); do ln -f overview/$f seq/ov_$(printf %05d $i).png; i=$((i+1)); done
ffmpeg -framerate 25 -i seq/ov_%05d.png -c:v libx264 -pix_fmt yuv420p -crf 20 spider_demo.mp4
```

## Tests
```bash
cd ~/spider_v2_ws/src/spider_sim && python3 -m pytest -q test
```

## What was fixed (v1 zip -> v2)
- `setup.py` had no `entry_points`, so none of the nodes could start
- the IK node was named `arm_controller`, the same as the ros2_control controller (now `arm_ik`)
- leg `yaw` was never applied in the URDF: legs now splay out at ±45° / ±135°
- gait used left/right pairs and only moved the femur, so it could not walk; now a real diagonal trot that walks and turns
- gait overwrote manual leg commands every 50 ms; now goes quiet when stopped
- arm IK shoulder sign was inverted against the URDF joint axes (tests in `test/`)
- `/clock` was never bridged although `use_sim_time` was on; all nodes now use sim time
- no hard-coded `/opt/ros/jazzy` path; controllers/nodes start after spawn, not on fixed timers
- recording cameras + demo script added; obstacle moved to x = 1.6 m so the walk has room

## Known limits
- the gripper only publishes GRABBED / RELEASED, nothing is physically picked up
- the lidar link is visual only (no sensor)
- with rendering on, the sim runs slower than real time on a laptop
