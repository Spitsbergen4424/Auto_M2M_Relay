# GFS-X test version

This copy is intentionally separate from the original project.

## Policy interface

- 15 vector observations, one stack.
- 3 continuous actions: throttle, steering, and horizontal camera servo.
- 1 discrete branch with two values: idle and grab.
- The LSTM in `config.yaml` provides temporal memory, so four observation stacks are no longer duplicated.

The changed interface is incompatible with ONNX/checkpoint files trained with the old 15 x 4 / 3 + [3] layout because stacking and the discrete branch changed. Start a new run ID.

## Reward and diagnostics

- Blind movement no longer receives privileged ground-truth distance guidance.
- New explored grid cells and new camera sectors receive small one-time rewards.
- First visual detection receives +0.25 and reaching the gripper sensor receives +0.5.
- Distance progress and alignment are rewarded only while the ball is visible.
- Camera visibility ignores the observing robot's own colliders but still respects walls and obstacles.
- A successful grab adds +5 instead of replacing the current decision reward.
- Collision penalties apply only to `Obstacle_*` and `Wall_*` objects.
- TensorBoard also receives ball discovery rate, detection time, and arena difficulty.

## Arenas

- Random obstacle layouts are checked on a grid. Optional obstacles are removed when they block every path.
- Ball direction expands from a visible 50-degree sector to the full 360 degrees.
- The curriculum progresses from zero obstacles to 4-7 random obstacles and occasional occluding barriers.
- Curiosity reward (`strength: 0.02`) supports exploration without overwhelming the +5 task reward.
- `SampleScene` is the training scene.
- `EvaluationScene` uses held-out obstacle seeds and behavior name `GFSX_Evaluation`. Assign a trained ONNX model to its Behavior Parameters and run it without `mlagents-learn` to measure generalization.

## Suggested first training run

```bash
mlagents-learn config.yaml --run-id=gfsx_test_v1 --env=/path/to/GFSX_Simulator.x86_64 --no-graphics
```

Do not use `--resume` with an old run because the observation and action shapes changed.
