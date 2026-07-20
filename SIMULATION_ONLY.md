# Simulation-only configuration

This branch is intended for Unity ML-Agents training and local inference.

- `SimulatedYoloCamera` calculates ball visibility inside Unity.
- The real YOLO UDP receiver was removed.
- `ROSBridge`, `ROSConnection`, and the ROS-TCP-Connector package were removed.
- Virtual sensors remain active and are calculated by the Unity simulation.

Assign an ONNX model in the robot's `Behavior Parameters` and set `Behavior Type`
to `Inference Only` for a visual test run.
