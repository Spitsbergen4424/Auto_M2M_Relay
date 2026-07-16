using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody), typeof(TrackController), typeof(VirtualSensors))]
public sealed class RobotBrain : Agent
{
    [Header("Robot subsystems")]
    [SerializeField] private TrackController trackController;
    [SerializeField] private VirtualSensors virtualSensors;
    [SerializeField] private GripperController gripperController;
    [SerializeField] private SimulatedYoloCamera yoloCamera;
    [SerializeField] private Transform cameraPivot;
    [SerializeField] private Transform targetBall;
    [SerializeField] private EnvironmentRandomizer environmentRandomizer;

    [Header("Episode")]
    // Used to normalize the position observations (11/12) and, only when no
    // EnvironmentRandomizer is wired, as a fallback out-of-bounds radius. The real
    // out-of-bounds check normally queries the randomizer's actual floor footprint instead
    // (see IsOutOfBounds) - it used to be this same fixed radius, which silently broke once
    // ball_max_distance grew past it: the agent got penalized for legitimately driving far
    // enough to reach the ball.
    [SerializeField] private float arenaRadius = 6.5f;
    [SerializeField] private float cameraServoSpeed = 90f;
    [SerializeField] private float cameraServoLimit = 70f;

    private Rigidbody body;
    private Rigidbody ballBody;
    private Vector3 robotStartPosition;
    private Quaternion robotStartRotation;
    private float cameraYaw;
    private float previousDistance;
    private Vector2 previousDriveAction;
    private bool initialized;
    private float baseMass;
    private bool isTraining;

    // Domain randomization (training only, see P5 guide): simulates the YOLO camera losing
    // the ball for several decision steps during a fast turn, and a FIFO command queue
    // simulating Wi-Fi/ROS control latency. Both are cosmetic to the sim and only exist to
    // stop the policy from relying on perfectly instantaneous, noise-free perception/control
    // that the real robot cannot provide.
    private int burstDropoutRemaining;
    private readonly Queue<float[]> actionBuffer = new Queue<float[]>();
    private int currentActionLatency;

    private void Update()
    {
        if (!IsManualControl())
        {
            return;
        }

        Keyboard keyboard = Keyboard.current;
        if (keyboard == null)
        {
            trackController?.SetCommand(0f, 0f);
            return;
        }

        float gas = Axis(keyboard.wKey.isPressed, keyboard.sKey.isPressed);
        float steer = Axis(keyboard.dKey.isPressed, keyboard.aKey.isPressed);
        trackController?.SetCommand(gas, steer);
        float cameraPan = Axis(keyboard.eKey.isPressed, keyboard.qKey.isPressed);
        ApplyCameraYaw(cameraPan * cameraServoSpeed * Time.deltaTime);

        // Poll these every rendered frame instead of once per ML-Agents decision.
        // Holding the key is intentional: capture also succeeds as soon as the ball enters range.
        if (keyboard.rKey.isPressed)
        {
            gripperController?.Release();
        }
        else if (keyboard.spaceKey.isPressed)
        {
            gripperController?.TryGrab();
        }
    }

    public override void Initialize()
    {
        body = GetComponent<Rigidbody>();
        baseMass = body.mass;
        trackController ??= GetComponent<TrackController>();
        virtualSensors ??= GetComponent<VirtualSensors>();
        gripperController ??= GetComponent<GripperController>();
        // In Assets/Prefab/Scene.prefab, GFSX_Robot and TrainingArena (which carries the
        // randomizer) are siblings under one shared "Scene" root, not ancestor/descendant -
        // so GetComponentInParent alone never finds it. Searching the shared parent's own
        // children stays correctly scoped to *this* training area even with dozens of
        // Scene.prefab copies side by side, unlike a scene-wide search which would return
        // an arbitrary copy's randomizer for every agent. The scene-wide fallback only
        // covers ad-hoc setups with no shared parent at all (e.g. a lone robot dropped
        // straight into a scene for a quick test).
        environmentRandomizer ??= GetComponentInParent<EnvironmentRandomizer>() ??
                                   transform.parent?.GetComponentInChildren<EnvironmentRandomizer>() ??
                                   FindFirstObjectByType<EnvironmentRandomizer>();
        if (targetBall != null)
        {
            ballBody = targetBall.GetComponent<Rigidbody>();
        }

        robotStartPosition = transform.position;
        robotStartRotation = transform.rotation;

        initialized = true;
    }

    public void Configure(TrackController tracks, VirtualSensors sensors, GripperController gripper,
        SimulatedYoloCamera yolo, Transform servoPivot, Transform ball)
    {
        trackController = tracks;
        virtualSensors = sensors;
        gripperController = gripper;
        yoloCamera = yolo;
        cameraPivot = servoPivot;
        targetBall = ball;
    }

    public override void OnEpisodeBegin()
    {
        if (!initialized)
        {
            Initialize();
        }

        gripperController?.Release();
        trackController?.Stop();
        body.linearVelocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;

        isTraining = Academy.IsInitialized && Academy.Instance.IsCommunicatorOn;
        burstDropoutRemaining = 0;
        if (isTraining)
        {
            // Randomize mass/drive dynamics (P5): different battery/gearbox wear per
            // episode, so the policy learns a robust drive response instead of one exact
            // motor curve. 0.4x-1.6x mirrors the guide's ~1.0-4.0kg range around a 2.5kg base.
            body.mass = Random.Range(baseMass * 0.4f, baseMass * 1.6f);
            trackController?.RandomizeDynamics();
            currentActionLatency = Random.Range(8, 14);
        }
        else
        {
            body.mass = baseMass;
            trackController?.ResetDynamics();
            currentActionLatency = 0;
        }

        actionBuffer.Clear();
        for (int i = 0; i < currentActionLatency; i++)
        {
            actionBuffer.Enqueue(new float[] { 0f, 0f, 0f });
        }

        if (targetBall != null)
        {
            targetBall.SetParent(null, true);
            if (ballBody != null)
            {
                ballBody.isKinematic = false;
            }

            foreach (Collider item in targetBall.GetComponentsInChildren<Collider>(true))
            {
                item.enabled = true;
            }
        }

        // Domain randomization owns robot/ball/obstacle placement so this keeps working
        // no matter how the reward/observation code around it changes later.
        environmentRandomizer?.Randomize();
        robotStartPosition = transform.position;
        robotStartRotation = transform.rotation;

        cameraYaw = 0f;
        if (cameraPivot != null)
        {
            cameraPivot.localRotation = Quaternion.identity;
        }

        previousDistance = DistanceToBall();
        previousDriveAction = Vector2.zero;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        float rawUltrasonic = virtualSensors != null ? virtualSensors.UltrasonicNormalized : 1f;
        // Domain randomization (training only): a real HC-SR04 jitters by a few centimetres
        // off uneven/angled walls. Without this, the policy would trust millimetre-precise
        // sim distances that the real sensor can never provide.
        float ultrasonicNoise = isTraining ? Random.Range(-0.05f, 0.05f) : 0f;
        float ultrasonic = Mathf.Clamp01(rawUltrasonic + ultrasonicNoise);
        float leftIr = virtualSensors != null ? virtualSensors.LeftIR : 0f;
        float rightIr = virtualSensors != null ? virtualSensors.RightIR : 0f;
        float gripperIr = virtualSensors != null ? virtualSensors.GripperIR : 0f;

        // Domain randomization (training only): simulate the real YOLO camera losing the
        // ball for several decision steps during a fast turn (motion blur) - a 15% chance per
        // decision while turning quickly, lasting 5-15 decisions, forces the policy to use
        // LastKnownDirection/memory instead of assuming the ball is always instantly visible.
        if (burstDropoutRemaining > 0)
        {
            burstDropoutRemaining--;
        }
        else if (isTraining && body.angularVelocity.magnitude > 0.5f && Random.value < 0.15f)
        {
            burstDropoutRemaining = Random.Range(5, 16);
        }

        bool visible = yoloCamera != null && yoloCamera.IsVisible && burstDropoutRemaining <= 0;

        sensor.AddObservation(ultrasonic);                                             // 1
        sensor.AddObservation(leftIr);                                                // 2
        sensor.AddObservation(rightIr);                                               // 3
        sensor.AddObservation(gripperIr);                                             // 4
        sensor.AddObservation(visible ? yoloCamera.HorizontalOffset : 0f);             // 5
        sensor.AddObservation(visible ? yoloCamera.NormalizedDistance : 1f);           // 6
        sensor.AddObservation(yoloCamera != null ? yoloCamera.LastKnownDirection : 0f);// 7
        sensor.AddObservation(visible ? 1f : 0f);                                      // 8
        sensor.AddObservation(cameraServoLimit > 0f ? cameraYaw / cameraServoLimit : 0f);// 9
        sensor.AddObservation(gripperController != null && gripperController.HasBall ? 1f : 0f);// 10
        Vector3 relative = transform.position - robotStartPosition;
        sensor.AddObservation(Mathf.Clamp(relative.x / arenaRadius, -1f, 1f));          // 11
        sensor.AddObservation(Mathf.Clamp(relative.z / arenaRadius, -1f, 1f));          // 12
        sensor.AddObservation(Mathf.DeltaAngle(robotStartRotation.eulerAngles.y,
            transform.eulerAngles.y) / 180f);                                         // 13
        float maxSpeed = trackController != null ? trackController.MaxLinearSpeed : 1f;
        sensor.AddObservation(maxSpeed > 0f
            ? Mathf.Clamp01(body.linearVelocity.magnitude / maxSpeed) : 0f);            // 14
        sensor.AddObservation(yoloCamera != null ? Mathf.Clamp01(yoloCamera.TimeSinceDetection / 5f) : 1f);// 15
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float rawGas = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float rawSteer = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float rawCameraInput = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        float gas, steer, cameraInput;
        if (isTraining && currentActionLatency > 0)
        {
            // Domain randomization (training only): route the fresh action through a FIFO
            // queue so the robot acts on a command issued 8-13 physics steps ago (~160-260ms
            // at the real Wi-Fi/ROS control loop's typical ping), instead of reacting
            // instantly like only the simulation can.
            actionBuffer.Enqueue(new[] { rawGas, rawSteer, rawCameraInput });
            float[] delayed = actionBuffer.Dequeue();
            gas = delayed[0];
            steer = delayed[1];
            cameraInput = delayed[2];
        }
        else
        {
            gas = rawGas;
            steer = rawSteer;
            cameraInput = rawCameraInput;
        }

        if (!IsManualControl())
        {
            trackController.SetCommand(gas, steer);
            ApplyCameraYaw(cameraInput * cameraServoSpeed * Time.fixedDeltaTime);

            int gripperCommand = actions.DiscreteActions.Length > 0 ? actions.DiscreteActions[0] : 0;
            gripperController?.ApplyCommand(gripperCommand);
        }

        CalculateRewards(gas, steer);
    }

    private void ApplyCameraYaw(float degreesDelta)
    {
        if (cameraPivot == null)
        {
            return;
        }

        cameraYaw = Mathf.Clamp(cameraYaw + degreesDelta, -cameraServoLimit, cameraServoLimit);
        cameraPivot.localRotation = Quaternion.Euler(0f, cameraYaw, 0f);
    }

    private bool IsManualControl()
    {
        BehaviorParameters behavior = GetComponent<BehaviorParameters>();
        return behavior != null && behavior.IsInHeuristicMode();
    }

    private void CalculateRewards(float gas, float steer)
    {
        float distance = DistanceToBall();
        // Clamp before scaling: a ball/obstacle spawn overlap can make PhysX eject the ball
        // at extreme speed for a step or two while it resolves, which would otherwise show up
        // as a several-hundred-unit distanceDelta and blow up the reward for that step alone.
        // Legitimate movement between decisions is well under 1 unit, so this only ever
        // clips genuine physics glitches, not real approach/retreat behaviour.
        float distanceDelta = Mathf.Clamp(previousDistance - distance, -2f, 2f);
        float proximityScale = distance < 0.6f ? 2f : 1f;
        AddReward(distanceDelta * 0.6f * proximityScale);

        Vector2 action = new Vector2(gas, steer);
        AddReward(-Vector2.Distance(action, previousDriveAction) * 0.0025f);
        previousDriveAction = action;

        if (yoloCamera != null && yoloCamera.IsVisible)
        {
            AddReward((1f - Mathf.Abs(yoloCamera.HorizontalOffset)) * 0.0008f);
        }

        if (targetBall != null)
        {
            Vector3 toBall = Vector3.ProjectOnPlane(targetBall.position - transform.position, Vector3.up);
            if (toBall.sqrMagnitude > 0.0001f)
            {
                // Ground-truth bearing to the ball, independent of whether the camera can
                // currently see it. With the ball now spawning anywhere in a 360 degree ring
                // (and often out of camera view at episode start), this is the only signal
                // that teaches "turn toward the ball" before it is ever spotted - weighted
                // well above the old value so search behaviour actually gets learned instead
                // of drowned out by the other small shaping terms.
                // The FBX nose points along local +X (transform.right).
                float alignment = Mathf.Clamp01((Vector3.Dot(transform.right, toBall.normalized) + 1f) * 0.5f);
                AddReward(alignment * 0.003f);
            }
        }

        if (virtualSensors != null)
        {
            AddReward(-(virtualSensors.LeftIR + virtualSensors.RightIR) * 0.002f);
            if (virtualSensors.UltrasonicNormalized < 0.08f)
            {
                AddReward(-0.003f);
            }
        }

        AddReward(-0.0002f);
        previousDistance = distance;

        if (gripperController != null && gripperController.HasBall)
        {
            // Manual play must retain the ball until R is pressed. Training still ends the
            // episode as soon as the capture succeeds.
            if (!IsManualControl())
            {
                SetReward(5f);
                EndEpisode();
            }
        }
        else if (IsOutOfBounds() || transform.position.y < robotStartPosition.y - 1f)
        {
            AddReward(-2f);
            EndEpisode();
        }
    }

    private bool IsOutOfBounds()
    {
        // Prefer the real floor footprint over a fixed radius from the start point - the
        // start point itself is now randomized anywhere in the arena, and the ball can be
        // placed far enough away that "distance from start" alone would flag perfectly
        // legitimate driving as an escape.
        if (environmentRandomizer != null && environmentRandomizer.TryGetFloorBounds(out Vector2 min, out Vector2 max))
        {
            const float escapeMargin = 0.5f; // wall thickness + collision slop before it counts as a real escape
            Vector3 position = transform.position;
            return position.x < min.x - escapeMargin || position.x > max.x + escapeMargin ||
                   position.z < min.y - escapeMargin || position.z > max.y + escapeMargin;
        }

        return (transform.position - robotStartPosition).sqrMagnitude > arenaRadius * arenaRadius;
    }

    private float DistanceToBall()
    {
        return targetBall != null ? Vector3.Distance(transform.position, targetBall.position) : arenaRadius;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuous = actionsOut.ContinuousActions;
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null)
        {
            continuous[0] = 0f;
            continuous[1] = 0f;
            continuous[2] = 0f;
            return;
        }

        continuous[0] = Axis(keyboard.wKey.isPressed, keyboard.sKey.isPressed);
        continuous[1] = Axis(keyboard.dKey.isPressed, keyboard.aKey.isPressed);
        continuous[2] = 0f;
        if (actionsOut.DiscreteActions.Length > 0)
        {
            ActionSegment<int> discrete = actionsOut.DiscreteActions;
            discrete[0] = 0;
        }
    }

    private static float Axis(bool positive, bool negative)
    {
        return (positive ? 1f : 0f) - (negative ? 1f : 0f);
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (!collision.collider.CompareTag("TargetBall"))
        {
            AddReward(-0.01f);
        }
    }
}
