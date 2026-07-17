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
    [SerializeField] private DiagnosticLogger diagnosticLogger;

    [Header("Episode")]
    [SerializeField] private float arenaRadius = 4.5f;
    [SerializeField] private float successReward = 5f;
    [SerializeField] private float cameraServoSpeed = 90f;
    [SerializeField] private float cameraServoLimit = 70f;

    private Rigidbody body;
    private Rigidbody ballBody;
    private Vector3 robotStartPosition;
    private Quaternion robotStartRotation;
    private Vector3 ballStartPosition;
    private Quaternion ballStartRotation;
    private float cameraYaw;
    private float cameraTurnCommand;
    private float previousDistance;
    private Vector2 previousDriveAction;
    private ArenaObstacleRandomizer obstacleRandomizer;
    private bool initialized;
    private bool episodeRunning;
    private float episodeStartTime;
    private float episodeTravelDistance;
    private float episodeInitialDistance;
    private Vector3 previousTravelPosition;
    private int episodeCollisionCount;
    private readonly HashSet<Vector2Int> visitedCells = new HashSet<Vector2Int>();
    private readonly HashSet<int> initialScanSectors = new HashSet<int>();
    private readonly HashSet<int> movingViewpoints = new HashSet<int>();
    private bool ballEverSeen;
    private bool ballWasVisible;
    private bool gripperReached;
    private float detectionTime;
    private float episodeDifficulty;
    private float previousCameraError;
    private float previousAlignment;
    private bool hasVisualRewardBaseline;
    private float lastBallVisibleTime;
    private float searchWindowStartTime;
    private Vector3 searchWindowStartPosition;
    private int episodeStationarySpinSteps;
    private int episodeStuckEvents;
    private int episodeSearchCells;

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
        diagnosticLogger ??= GetComponent<DiagnosticLogger>();
        obstacleRandomizer = GetComponentInParent<ArenaObstacleRandomizer>();
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
        cameraTurnCommand = 0f;
        if (cameraPivot != null)
        {
            cameraPivot.localRotation = Quaternion.identity;
        }

        if (targetBall != null)
        {
            targetBall.SetParent(null, true);
            episodeDifficulty = Mathf.Clamp01(
                Academy.Instance.EnvironmentParameters.GetWithDefault("arena_difficulty", 1f));
            float halfAngle = Mathf.Lerp(25f, 180f, episodeDifficulty);
            float angle = Random.Range(-halfAngle, halfAngle);
            float minimumDistance = Mathf.Lerp(1.1f, 1.8f, episodeDifficulty);
            float maximumDistance = Mathf.Lerp(2.0f, 4.1f, episodeDifficulty);
            Vector3 spawnDirection = Quaternion.AngleAxis(angle, Vector3.up) * transform.right;
            Vector3 offset = spawnDirection.normalized * Random.Range(minimumDistance, maximumDistance);
            Vector3 spawnCenter = new Vector3(robotStartPosition.x, ballStartPosition.y, robotStartPosition.z);
            targetBall.SetPositionAndRotation(spawnCenter + offset, ballStartRotation);
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

        obstacleRandomizer?.RandomizeLayout();

        previousDistance = DistanceToBall();
        previousDriveAction = Vector2.zero;
        if (episodeRunning)
        {
            ReportEpisode(false);
        }

        episodeRunning = true;
        episodeStartTime = Time.time;
        episodeTravelDistance = 0f;
        episodeInitialDistance = previousDistance;
        previousTravelPosition = transform.position;
        episodeCollisionCount = 0;
        visitedCells.Clear();
        initialScanSectors.Clear();
        movingViewpoints.Clear();
        visitedCells.Add(Vector2Int.zero);
        ballEverSeen = false;
        ballWasVisible = false;
        gripperReached = false;
        detectionTime = -1f;
        previousCameraError = 0f;
        previousAlignment = 0f;
        hasVisualRewardBaseline = false;
        lastBallVisibleTime = episodeStartTime;
        searchWindowStartTime = episodeStartTime;
        searchWindowStartPosition = transform.position;
        episodeStationarySpinSteps = 0;
        episodeStuckEvents = 0;
        episodeSearchCells = 0;
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
        cameraTurnCommand = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        if (!IsManualControl())
        {
            trackController.SetCommand(gas, steer);

            int gripperCommand = actions.DiscreteActions.Length > 0 ? actions.DiscreteActions[0] : 0;
            gripperController?.ApplyCommand(gripperCommand);
        }

        CalculateRewards(gas, steer);
        LogDiagnosticStep(gas, steer);
    }

    private void LogDiagnosticStep(float gas, float steer)
    {
        if (diagnosticLogger == null)
        {
            return;
        }

        bool ballSeen = yoloCamera != null && yoloCamera.IsVisible;
        float ballAngle = ballSeen ? yoloCamera.HorizontalOffset : 0f;
        float ballDistance = ballSeen ? yoloCamera.NormalizedDistance : 1f;

        float ultrasonic = virtualSensors != null ? virtualSensors.UltrasonicNormalized : 1f;
        float leftIr = virtualSensors != null ? virtualSensors.LeftIR : 0f;
        float rightIr = virtualSensors != null ? virtualSensors.RightIR : 0f;
        float gripperIr = virtualSensors != null ? virtualSensors.GripperIR : 0f;

        bool hasBall = gripperController != null && gripperController.HasBall;
        Vector3 displacement = transform.position - robotStartPosition;
        float heading = Mathf.Repeat(transform.eulerAngles.y, 360f) / 360f;
        float speed = body != null ? body.linearVelocity.magnitude : 0f;

        const int holdTicks = 0;
        const bool isRetrying = false;

        diagnosticLogger.LogStep(
            StepCount,
            ballSeen,
            ballAngle,
            ballDistance,
            ultrasonic,
            leftIr,
            rightIr,
            gripperIr,
            cameraYaw,
            gas,
            steer,
            hasBall,
            holdTicks,
            isRetrying,
            displacement.x,
            displacement.z,
            heading,
            speed);
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
        bool visible = yoloCamera != null && yoloCamera.IsVisible;

        // Ground-truth distance must not guide a blind robot. Progress becomes rewarding
        // only after the camera actually sees the ball.
        if (visible)
        {
            lastBallVisibleTime = Time.time;
            searchWindowStartTime = Time.time;
            searchWindowStartPosition = transform.position;
            AddReward(distanceDelta * 0.6f);
            if (!ballEverSeen)
            {
                ballEverSeen = true;
                detectionTime = Time.time - episodeStartTime;
                AddReward(0.25f);
            }
        }
        else
        {
            Vector3 relative = transform.position - robotStartPosition;
            var cell = new Vector2Int(
                Mathf.RoundToInt(relative.x / ActiveSearchRewardShaping.SearchCellSize),
                Mathf.RoundToInt(relative.z / ActiveSearchRewardShaping.SearchCellSize));
            if (visitedCells.Add(cell))
            {
                episodeSearchCells++;
                AddReward(ActiveSearchRewardShaping.NewAreaReward);
            }

            float timeWithoutBall = Time.time - lastBallVisibleTime;
            Vector3 planarVelocity = Vector3.ProjectOnPlane(body.linearVelocity, Vector3.up);
            bool moving = planarVelocity.magnitude >= ActiveSearchRewardShaping.MinimumMovingSpeed;
            bool spinning = body.angularVelocity.magnitude >= ActiveSearchRewardShaping.SpinAngularSpeed;
            int cameraSector = yoloCamera != null ? yoloCamera.WorldViewSector : 0;

            if (timeWithoutBall <= ActiveSearchRewardShaping.InitialScanDuration &&
                initialScanSectors.Add(cameraSector))
            {
                AddReward(ActiveSearchRewardShaping.InitialSectorReward);
            }

            // After the initial look-around, scanning is useful only while relocating.
            // The cell/sector pair rewards new moving viewpoints without encouraging
            // stop-and-scan behaviour in every small grid cell.
            int viewpointKey = ActiveSearchRewardShaping.ViewpointKey(cell, cameraSector);
            if (moving && movingViewpoints.Add(viewpointKey))
            {
                AddReward(ActiveSearchRewardShaping.MovingViewpointReward);
            }

            if (ActiveSearchRewardShaping.ShouldPenalizeStationarySpin(
                    timeWithoutBall, planarVelocity.magnitude, body.angularVelocity.magnitude))
            {
                episodeStationarySpinSteps++;
                AddReward(-ActiveSearchRewardShaping.StationarySpinPenalty);
            }

            if (Time.time - searchWindowStartTime >= ActiveSearchRewardShaping.StuckWindowDuration)
            {
                float displacement = Vector3.ProjectOnPlane(
                    transform.position - searchWindowStartPosition, Vector3.up).magnitude;
                if (timeWithoutBall > ActiveSearchRewardShaping.InitialScanDuration &&
                    displacement < ActiveSearchRewardShaping.MinimumWindowDisplacement)
                {
                    episodeStuckEvents++;
                    AddReward(-ActiveSearchRewardShaping.StuckPenalty);
                }

                searchWindowStartTime = Time.time;
                searchWindowStartPosition = transform.position;
            }
        }

        if (ballWasVisible && !visible)
        {
            AddReward(-0.01f);
        }
        ballWasVisible = visible;

        Vector2 action = new Vector2(gas, steer);
        AddReward(-Vector2.Distance(action, previousDriveAction) * 0.0025f);
        previousDriveAction = action;

        if (visible && targetBall != null)
        {
            Vector3 toBall = Vector3.ProjectOnPlane(targetBall.position - transform.position, Vector3.up);
            if (toBall.sqrMagnitude > 0.0001f)
            {
                // The FBX nose points along local +X (transform.right).
                float alignment = Mathf.Clamp01((Vector3.Dot(transform.right, toBall.normalized) + 1f) * 0.5f);
                float cameraError = Mathf.Abs(yoloCamera.HorizontalOffset);
                if (hasVisualRewardBaseline)
                {
                    AddReward(VisualRewardShaping.CalculateProgress(
                        previousCameraError, cameraError, previousAlignment, alignment));
                }

                previousCameraError = cameraError;
                previousAlignment = alignment;
                hasVisualRewardBaseline = true;
            }
        }

        if (virtualSensors != null)
        {
            AddReward(-(virtualSensors.LeftIR + virtualSensors.RightIR) * 0.002f);
            if (virtualSensors.UltrasonicNormalized < 0.08f)
            {
                AddReward(-0.003f);
            }

            if (!gripperReached && virtualSensors.GripperIR >= 1f)
            {
                gripperReached = true;
                AddReward(0.5f);
            }
        }

        AddReward(-0.0004f);
        previousDistance = distance;

        if (gripperController != null && gripperController.HasBall)
        {
            // Manual play must retain the ball until R is pressed. Training still ends the
            // episode as soon as the capture succeeds.
            if (!IsManualControl())
            {
                AddReward(successReward);
                ReportEpisode(true);
                EndEpisode();
            }
        }
        else if ((transform.position - robotStartPosition).sqrMagnitude > arenaRadius * arenaRadius ||
                 transform.position.y < robotStartPosition.y - 1f)
        {
            AddReward(-2f);
            ReportEpisode(false);
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

    private void FixedUpdate()
    {
        UpdateCameraServo();
        if (!episodeRunning)
        {
            return;
        }

        episodeTravelDistance += Vector3.Distance(transform.position, previousTravelPosition);
        previousTravelPosition = transform.position;
    }

    private void UpdateCameraServo()
    {
        if (cameraPivot == null)
        {
            return;
        }

        float nextCameraYaw = Mathf.Clamp(
            cameraYaw + cameraTurnCommand * cameraServoSpeed * Time.fixedDeltaTime,
            -cameraServoLimit,
            cameraServoLimit);
        float deltaYaw = nextCameraYaw - cameraYaw;
        cameraYaw = nextCameraYaw;
        cameraPivot.Rotate(transform.up, deltaYaw, Space.World);
    }

    private void ReportEpisode(bool success)
    {
        if (!episodeRunning)
        {
            return;
        }

        float pathEfficiency = episodeTravelDistance > 0.001f
            ? Mathf.Clamp01(episodeInitialDistance / episodeTravelDistance)
            : 0f;
        StatsRecorder stats = Academy.Instance.StatsRecorder;
        stats.Add("Robot/SuccessRate", success ? 1f : 0f);
        stats.Add("Robot/EpisodeSeconds", Time.time - episodeStartTime);
        stats.Add("Robot/CollisionCount", episodeCollisionCount);
        stats.Add("Robot/TravelDistance", episodeTravelDistance);
        stats.Add("Robot/PathEfficiency", pathEfficiency);
        stats.Add("Robot/FoundBallRate", ballEverSeen ? 1f : 0f);
        stats.Add("Robot/DetectionSeconds", detectionTime >= 0f ? detectionTime : Time.time - episodeStartTime);
        stats.Add("Robot/ArenaDifficulty", episodeDifficulty, StatAggregationMethod.MostRecent);
        stats.Add("Robot/SearchCells", episodeSearchCells);
        stats.Add("Robot/StationarySpinSteps", episodeStationarySpinSteps);
        stats.Add("Robot/StuckEvents", episodeStuckEvents);
        episodeRunning = false;
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (IsObstacleOrWall(collision.collider.transform))
        {
            episodeCollisionCount++;
            AddReward(-0.03f);
        }
    }

    private static bool IsObstacleOrWall(Transform item)
    {
        for (Transform current = item; current != null; current = current.parent)
        {
            if (current.name.StartsWith("Obstacle_") || current.name.StartsWith("Wall_"))
            {
                return true;
            }
        }

        return false;
    }
}