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

    [Header("Episode")]
    [SerializeField] private float arenaRadius = 4.5f;
    [SerializeField] private float cameraServoSpeed = 90f;
    [SerializeField] private float cameraServoLimit = 70f;

    private Rigidbody body;
    private Rigidbody ballBody;
    private Vector3 robotStartPosition;
    private Quaternion robotStartRotation;
    private Vector3 ballStartPosition;
    private Quaternion ballStartRotation;
    private float cameraYaw;
    private float previousDistance;
    private Vector2 previousDriveAction;
    private bool initialized;

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
        trackController ??= GetComponent<TrackController>();
        virtualSensors ??= GetComponent<VirtualSensors>();
        gripperController ??= GetComponent<GripperController>();
        if (targetBall != null)
        {
            ballBody = targetBall.GetComponent<Rigidbody>();
        }

        robotStartPosition = transform.position;
        robotStartRotation = transform.rotation;
        if (targetBall != null)
        {
            ballStartPosition = targetBall.position;
            ballStartRotation = targetBall.rotation;
        }

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
        transform.SetPositionAndRotation(robotStartPosition, robotStartRotation);

        cameraYaw = 0f;
        if (cameraPivot != null)
        {
            cameraPivot.localRotation = Quaternion.identity;
        }

        if (targetBall != null)
        {
            targetBall.SetParent(null, true);
            Vector3 offset = Vector3.zero;
            if (!IsManualControl())
            {
                float angle = Random.Range(-65f, 65f) * Mathf.Deg2Rad;
                float distance = Random.Range(1.0f, 2.8f);
                offset = new Vector3(Mathf.Sin(angle), 0f, Mathf.Cos(angle)) * distance;
            }
            targetBall.SetPositionAndRotation(ballStartPosition + offset, ballStartRotation);
            if (ballBody != null)
            {
                ballBody.isKinematic = false;
                ballBody.linearVelocity = Vector3.zero;
                ballBody.angularVelocity = Vector3.zero;
            }

            foreach (Collider item in targetBall.GetComponentsInChildren<Collider>(true))
            {
                item.enabled = true;
            }
        }

        previousDistance = DistanceToBall();
        previousDriveAction = Vector2.zero;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        float ultrasonic = virtualSensors != null ? virtualSensors.UltrasonicNormalized : 1f;
        float leftIr = virtualSensors != null ? virtualSensors.LeftIR : 0f;
        float rightIr = virtualSensors != null ? virtualSensors.RightIR : 0f;
        float gripperIr = virtualSensors != null ? virtualSensors.GripperIR : 0f;
        bool visible = yoloCamera != null && yoloCamera.IsVisible;

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
        float gas = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float steer = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        if (!IsManualControl())
        {
            trackController.SetCommand(gas, steer);

            int gripperCommand = actions.DiscreteActions.Length > 0 ? actions.DiscreteActions[0] : 0;
            gripperController?.ApplyCommand(gripperCommand);
        }

        CalculateRewards(gas, steer);
    }

    private bool IsManualControl()
    {
        BehaviorParameters behavior = GetComponent<BehaviorParameters>();
        return behavior != null && behavior.IsInHeuristicMode();
    }

    private void CalculateRewards(float gas, float steer)
    {
        float distance = DistanceToBall();
        float distanceDelta = previousDistance - distance;
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
                // The FBX nose points along local +X (transform.right).
                float alignment = Mathf.Clamp01((Vector3.Dot(transform.right, toBall.normalized) + 1f) * 0.5f);
                AddReward(alignment * 0.0008f);
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
        else if ((transform.position - robotStartPosition).sqrMagnitude > arenaRadius * arenaRadius ||
                 transform.position.y < robotStartPosition.y - 1f)
        {
            AddReward(-2f);
            EndEpisode();
        }
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
