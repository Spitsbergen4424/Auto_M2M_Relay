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
9. Use the `GfsxRealRobotBridge` component menu and run `Prepare Gripper` only
   when the arm has room to move.
10. Place the robot so the tracks are off the floor before enabling motion.
11. Set `dryRun=false`, then set `enableMotorCommands=true`.
12. Use the component menu `Emergency Stop` or exit Play Mode to stop.

## Safety Notes

- `GfsxRealRobotBridge` ships with `dryRun=true` and `enableMotorCommands=false`.
- Normalized PPO actions are mapped through a `0.15` deadband to the robot's
  effective motor range. The hardware cannot produce arbitrarily slow motion:
  `MIN_MOTOR_PWM=35` corresponds to roughly `0.175 m/s` in the current driver.
- Linear ROS commands are capped at `0.25 m/s` and angular commands at `0.9 rad/s`.
- Unity adds a local ultrasonic stop at `0.30 m` when moving forward.
- The bridge stops on stale ROS sensor packets and stale YOLO packets.
- The bridge also stops if PPO actions stop arriving for `0.5 s`.
- The bridge publishes a zero twist on disable, destroy, and application quit when motor commands are enabled.
- A stable gripper IR signal latches `ballCaptured` and permanently stops drive commands
  until the operator explicitly resets the captured state.

## Sensor Model

- Real ultrasonic input uses the simulation's `2.0 m` contract: `0 = near`, `1 = 2 m or farther`.
- Only `/sensor/data` refreshes ultrasonic safety. PWM and the separate gripper topic
  cannot make an old ultrasonic reading look fresh.
- Real pose is estimated by dead reckoning from the actually sent `linear.x` and `angular.z`.
- This estimate has no encoders or odometry and will accumulate error.

## Gripper Ownership

- `PrepareGripper()` sends command `1` once.
- The bridge does not spam command `2` from PPO.
- The Raspberry Pi keeps the final close-and-lift action.

## Unity Setup

Use `Tools > URFU > Configure Real Robot Scene` to build or refresh the scene, then `Tools > URFU > Validate Real Robot Scene` to verify the configuration.

The setup assigns `Assets/GFSX_Brain.onnx`, selects `InferenceOnly`, and sets
`RobotBrain.MaxStep=0`. A physical mission must not restart an ML-Agents episode
and reset dead reckoning while the robot remains in place.

## Raspberry Pi safety patch

Unity's 0.30 m stop is an additional layer, not the primary motor safety layer.
Apply the code in `ROBOT_SIDE_SAFETY_PATCH.md` to the host copy of
`unity_master_team2.py` before enabling tracks on the floor.
