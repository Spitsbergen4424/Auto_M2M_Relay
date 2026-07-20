# GFS-X Real Robot Launch

This document covers the safe real-robot path for `Auto_M2M_Relay`.

## What this mode does

- Unity connects directly to the Raspberry Pi at `192.168.2.154:10000`.
- YOLO runs on the Windows laptop and sends packets to Unity over UDP `127.0.0.1:5005`.
- The local `ros1-docker` stack is for simulation only and is not required for the physical robot.
- `RobotBrain` stays enabled and keeps producing PPO actions.
- `GfsxRealRobotBridge` publishes `/cmd_vel`, `/cmd_camera_pan`, and only the one-time gripper prepare command `1` when requested.
- Final gripper closing remains owned by the Raspberry Pi, which already sends command `2` on stable gripper IR.

## Launch Sequence

1. Start `start_robot_team2.sh` on the Raspberry Pi.
2. Verify that `192.168.2.154:10000` is reachable from the Windows laptop.
3. Start YOLO with `gfsx-yolo/run_yolo.ps1`.
4. Open `Assets/Scenes/RealRobotScene.unity`.
5. Keep `dryRun=true` and `enableMotorCommands=false` for the first run.
6. Confirm that the real sensors update in the inspector.
7. Confirm that YOLO packets stay fresh.
8. Confirm that `RobotBrain` produces actions.
9. Place the robot so the tracks are off the floor before enabling motion.
10. Only then enable motor commands.
11. Use `EmergencyStop()` or exit Play Mode to stop.

## Safety Notes

- `GfsxRealRobotBridge` ships with `dryRun=true` and `enableMotorCommands=false`.
- Linear motion is clamped to `0.05 m/s` by default.
- Angular motion is clamped to about `0.3 rad/s` by default.
- Unity adds a local ultrasonic stop at `0.30 m` when moving forward.
- The bridge stops on stale ROS sensor packets and stale YOLO packets.
- The bridge publishes a zero twist on disable, destroy, and application quit when motor commands are enabled.

## Sensor Model

- Real sensor input is normalized to match the simulation contract: `0 = near`, `1 = far`.
- Real pose is estimated by dead reckoning from the actually sent `linear.x` and `angular.z`.
- This estimate has no encoders or odometry and will accumulate error.

## Gripper Ownership

- `PrepareGripper()` sends command `1` once.
- The bridge does not spam command `2` from PPO.
- The Raspberry Pi keeps the final close-and-lift action.

## Unity Setup

Use `Tools > URFU > Configure Real Robot Scene` to build or refresh the scene, then `Tools > URFU > Validate Real Robot Scene` to verify the configuration.
