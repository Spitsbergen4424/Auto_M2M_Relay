# Real GFS-X robot: ROS1 connection

This project uses the ROS-TCP Endpoint already started by the robot script
`start_robot_team1.sh` on TCP port `10000`. It does **not** use a WebSocket
`rosbridge_server` on port 9090.

## Data flow

```text
Unity ROSBridge -- /cmd_vel, /cmd_gripper, /cmd_camera_pan --> unity_master_team1.py
unity_master_team1.py -- /sensor/data (Quaternion) --> gfsx_ros1_adapter.py
gfsx_ros1_adapter.py -- /gfsx sensors --> Unity ROSBridge
Windows yolo_vision_node.py -- UDP :5005 --> RealVision.cs --> Unity camera observations
```

The physical packet `/sensor/data` has this fixed layout:

| Field | Meaning |
| --- | --- |
| `x` | ultrasonic distance in metres |
| `y` | left IR obstacle sensor, 0/1 |
| `z` | right IR obstacle sensor, 0/1 |
| `w` | gripper IR sensor, 0/1 |

## Install the adapter on Raspberry Pi

Copy `ros1_ws/src/gfsx_unity_bridge` to `~/catkin_ws/src/`, then run:

```bash
cd ~/catkin_ws
chmod +x src/gfsx_unity_bridge/scripts/gfsx_ros1_adapter.py
catkin_make
source devel/setup.bash
```

Start the robot's original `start_robot_team1.sh` first. It starts `roscore`,
ROS-TCP Endpoint on port `10000`, and `unity_master_team1.py`.

Then start only the adapter:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch gfsx_unity_bridge gfsx_unity.launch
```

Do not start a second ROS-TCP Endpoint: the robot startup script already owns
port `10000`.

## Important gripper change

For learned control, do **not** launch `unity_gripper_ir_team1.py` from
`start_robot_team1.sh`: it automatically publishes its own commands to
`/cmd_gripper` and conflicts with Unity. Comment out the `4.5/4` launch block.

Unity now uses the physical protocol correctly:

| Unity action | Robot command | Result |
| --- | --- | --- |
| grab with empty gripper sensor | `1` | lower arm and open claw |
| gripper IR detects the ball after preparation | `2` | close claw and lift arm |
| release | `4` | open claw |

## Camera and YOLO (P7)

Unity sends camera pan normalized to `-1..1`; the robot converts it to the
servo range `0..180` with centre at 90.

The archive contains an MJPEG stream at `http://<robot-ip>:8080/`, but no ball
detector. `yolo_vision_node.py` is included in this project and runs on the
Windows PC. It sends UDP directly to `RealVision.cs`; it does not publish ROS
topics.

Install the PC packages once:

```cmd
py -m pip install -r requirements-yolo.txt
```

Run it from the project folder, replacing both paths:

```cmd
py yolo_vision_node.py --stream-url http://ROBOT_IP:8080/ --model C:\path\to\best_detect.pt
```

The script uses ball class `0`, selects the largest detected ball, and sends
`angle`, `distance`, `sees`, confidence and box size to UDP port `5005`.
Windows Firewall must allow incoming UDP on this local port.

## Safety test before driving

Lift the wheels from the floor first. Check that sensor data arrives:

```bash
rostopic echo /sensor/data
rostopic echo /gfsx/ultrasonic
rostopic echo /gfsx/gripper_ir
```

Unity stops forward commands at an ultrasonic reading of `0.50 m`; the
robot-side `unity_master_team1.py` watchdog stops motors after 0.5 seconds
without `/cmd_vel`.
