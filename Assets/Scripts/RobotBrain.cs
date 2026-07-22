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
    public int SuccessfulEpisodeCount { get; private set; }
    public int EpisodeSequence { get; private set; }

    [Header("Robot subsystems")]
    [SerializeField] private TrackController trackController;
    [SerializeField] private VirtualSensors virtualSensors;
    [SerializeField] private GripperController gripperController;
    [SerializeField] private SimulatedYoloCamera yoloCamera;
    [SerializeField] private DiagnosticLogger diagnosticLogger;
    [SerializeField] private Transform cameraPivot;
    [SerializeField] private Transform targetBall;

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
    private Vector3 previousAction;
    private ArenaObstacleRandomizer obstacleRandomizer;
    private bool initialized;
    private bool episodeRunning;
    private float episodeStartTime;
    private float episodeTravelDistance;
    private float episodeInitialDistance;
    private Vector3 previousTravelPosition;
    private int episodeCollisionCount;
    private readonly HashSet<Vector2Int> visitedCells = new HashSet<Vector2Int>();
    private bool ballEverSeen;
    private bool gripperReached;
    private float detectionTime;
    private float episodeDifficulty;
    private float previousNormalizedBallDistance;
    private bool hasApproachBaseline;
    private bool wasCloseToBall;
    private float blindPhaseDeadline;
    private float stuckWindowStartTime;
    private Vector3 stuckWindowStartPosition;
    private float stuckGasSum;
    private float stuckSteerSum;
    private int stuckSampleCount;
    private bool episodeEndedStuck;
    private int episodeSearchCells;
    private bool episodeDetourLayout;
    private bool hasDetourBaseline;
    private float previousDetourPotential;
    private bool usingSuccessGatedCurriculum;
    private ArenaEpisodeSetup episodeSetup;
    // Per-episode reward decomposition, reported as Robot/Reward/* so the balance
    // of each component is visible in TensorBoard instead of being reconstructed
    // from the cumulative total by hand.
    private float episodeRewardApproach;
    private float episodeRewardBonuses;
    private float episodeRewardObstaclePenalty;
    private float episodeRewardActionRatePenalty;
    private float episodeRewardOtherPenalties;
    private float episodeRewardTerminal;
    private float episodeRewardDetour;
    private float evaluationBoundaryRadiusOverride;

    public void SetEvaluationBoundaryRadius(float radius)
    {
        evaluationBoundaryRadiusOverride = Mathf.Max(0f, radius);
    }

    public void ResetEvaluationActuators()
    {
        trackController?.Stop();
        if (body != null)
        {
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
        }

        cameraYaw = 0f;
        cameraTurnCommand = 0f;
        previousAction = Vector3.zero;
        if (cameraPivot != null)
        {
            cameraPivot.localRotation = Quaternion.identity;
        }
    }

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
        EpisodeSequence++;
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
            float trainerDifficulty = Academy.Instance.EnvironmentParameters.GetWithDefault(
                "arena_difficulty", -1f);
            usingSuccessGatedCurriculum = trainerDifficulty < 0f && Academy.Instance.IsCommunicatorOn;
            if (trainerDifficulty >= 0f)
            {
                // An explicit yaml value (fixed or trainer-side curriculum) always wins.
                episodeSetup = ArenaEpisodeSetup.FromScalar(trainerDifficulty);
            }
            else if (usingSuccessGatedCurriculum)
            {
                // Training without environment_parameters in the yaml: the staged
                // ladder advances one axis at a time, and only after the policy
                // both plateaus and clears the competence bar (SuccessGatedCurriculum).
                episodeSetup = SuccessGatedCurriculum.CurrentSetup;
            }
            else
            {
                // Standalone Play without a trainer keeps the historical maximum.
                episodeSetup = ArenaEpisodeSetup.FromScalar(1f);
            }

            episodeDifficulty = episodeSetup.NormalizedDifficulty;
            float halfAngle = episodeSetup.SpawnHalfAngleDegrees;
            float angle = Random.Range(-halfAngle, halfAngle);
            float minimumDistance = episodeSetup.MinSpawnDistance;
            float maximumDistance = episodeSetup.MaxSpawnDistance;
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

        obstacleRandomizer?.RandomizeLayout(episodeSetup);
        episodeDetourLayout = obstacleRandomizer != null && obstacleRandomizer.HasDetourLayout;
        hasDetourBaseline = false;
        previousDetourPotential = 0f;

        previousAction = Vector3.zero;
        if (episodeRunning)
        {
            ReportEpisode(false);
        }

        episodeRunning = true;
        episodeStartTime = Time.time;
        episodeTravelDistance = 0f;
        episodeInitialDistance = DistanceToBall();
        previousTravelPosition = transform.position;
        episodeCollisionCount = 0;
        visitedCells.Clear();
        visitedCells.Add(Vector2Int.zero);
        ballEverSeen = false;
        gripperReached = false;
        detectionTime = -1f;
        previousNormalizedBallDistance = 1f;
        hasApproachBaseline = false;
        wasCloseToBall = false;
        blindPhaseDeadline = 0f;
        ResetStuckWindow();
        episodeEndedStuck = false;
        episodeSearchCells = 0;
        episodeRewardApproach = 0f;
        episodeRewardBonuses = 0f;
        episodeRewardObstaclePenalty = 0f;
        episodeRewardActionRatePenalty = 0f;
        episodeRewardOtherPenalties = 0f;
        episodeRewardTerminal = 0f;
        episodeRewardDetour = 0f;
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

        bool visible = yoloCamera != null && yoloCamera.IsVisible;
        Vector3 displacement = transform.position - robotStartPosition;
        diagnosticLogger.LogStep(
            StepCount,
            visible,
            visible ? yoloCamera.HorizontalOffset : 0f,
            visible ? yoloCamera.NormalizedDistance : 1f,
            virtualSensors != null ? virtualSensors.UltrasonicNormalized : 1f,
            virtualSensors != null ? virtualSensors.LeftIR : 0f,
            virtualSensors != null ? virtualSensors.RightIR : 0f,
            virtualSensors != null ? virtualSensors.GripperIR : 0f,
            cameraYaw,
            gas,
            steer,
            gripperController != null && gripperController.HasBall,
            0,
            wasCloseToBall && Time.time <= blindPhaseDeadline,
            displacement.x,
            displacement.z,
            Mathf.Repeat(transform.eulerAngles.y, 360f) / 360f,
            body != null ? body.linearVelocity.magnitude : 0f);
    }

    private bool IsManualControl()
    {
        BehaviorParameters behavior = GetComponent<BehaviorParameters>();
        return behavior != null && behavior.IsInHeuristicMode();
    }

    // Every reward flows through here so each component lands in its named
    // TensorBoard bucket. If Robot/Reward/Total ever drifts away from
    // Environment/Cumulative Reward, some code path is calling AddReward directly.
    private void AddTrackedReward(ref float bucket, float value)
    {
        bucket += value;
        AddReward(value);
    }

    private void CalculateRewards(float gas, float steer)
    {
        bool visible = yoloCamera != null && yoloCamera.IsVisible;
        bool blindPhaseActive = wasCloseToBall && Time.time <= blindPhaseDeadline;

        if (visible)
        {
            float ballDistance = yoloCamera.NormalizedDistance;

            // Camera-space approach shaping: the multiplier rises from 2x to 6x as
            // the ball nears, paying most for a precise final approach. It uses the
            // same normalized-distance signal the real YOLO pipeline produces, and
            // pays only on visual confirmation - ground truth never guides a blind
            // robot. The baseline survives blind gaps, so driving away unseen and
            // returning nets nothing while a genuine occlusion bypass keeps its
            // earned progress.
            if (hasApproachBaseline)
            {
                float approachDelta = previousNormalizedBallDistance - ballDistance;
                AddTrackedReward(ref episodeRewardApproach,
                    approachDelta * RewardTuning.ProximityMultiplier(ballDistance));
            }

            previousNormalizedBallDistance = ballDistance;
            hasApproachBaseline = true;

            if (!ballEverSeen)
            {
                ballEverSeen = true;
                detectionTime = Time.time - episodeStartTime;
                AddTrackedReward(ref episodeRewardBonuses, RewardTuning.FirstDetectionReward);
            }

            if (ballDistance < RewardTuning.CloseBallDistance)
            {
                // Remember the close encounter: when the ball drops under the
                // bumper out of the camera frame, the blind-crawl window lets the
                // robot finish the approach from memory without reward starvation.
                wasCloseToBall = true;
                blindPhaseDeadline = Time.time + RewardTuning.BlindPhaseSeconds;
                if (gas > RewardTuning.SlowGasMin && gas < RewardTuning.SlowGasMax)
                {
                    AddTrackedReward(ref episodeRewardBonuses, RewardTuning.SlowApproachBonus);
                }
            }
            else
            {
                wasCloseToBall = false;
            }

            // Centered-and-advancing only: an idle robot parked in front of the
            // ball must not farm the alignment stream.
            if (ballDistance < RewardTuning.AlignmentDistance &&
                Mathf.Abs(yoloCamera.HorizontalOffset) < RewardTuning.AlignmentAngle &&
                gas > RewardTuning.SlowGasMin)
            {
                AddTrackedReward(ref episodeRewardBonuses, RewardTuning.AlignmentBonus);
            }

            if (ballDistance < RewardTuning.NearBallSpeedDistance &&
                Mathf.Abs(gas) > RewardTuning.NearBallSpeedGas)
            {
                AddTrackedReward(ref episodeRewardOtherPenalties, -RewardTuning.NearBallSpeedPenalty);
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
                AddTrackedReward(ref episodeRewardBonuses, ActiveSearchRewardShaping.NewAreaReward);
            }

            // Directed exploration around an occluding barrier. Only active when a
            // barrier layout exists (final curriculum stages) and the ball is
            // hidden, so stages without occlusion are untouched. Potential-based:
            // moving toward the nearer free edge pays, backtracking refunds it.
            if (episodeDetourLayout && obstacleRandomizer != null &&
                obstacleRandomizer.TryGetDetourPathPotential(transform.position, out float detourPotential))
            {
                if (hasDetourBaseline)
                {
                    float progress = Mathf.Clamp(
                        previousDetourPotential - detourPotential,
                        -RewardTuning.MaxDetourProgressPerStep,
                        RewardTuning.MaxDetourProgressPerStep);
                    AddTrackedReward(ref episodeRewardDetour, progress * RewardTuning.DetourPathProgress);
                }

                previousDetourPotential = detourPotential;
                hasDetourBaseline = true;
            }

            if (blindPhaseActive &&
                gas > RewardTuning.SlowGasMin && gas < RewardTuning.SlowGasMax)
            {
                AddTrackedReward(ref episodeRewardBonuses, RewardTuning.BlindCrawlBonus);
            }

            if (!blindPhaseActive)
            {
                wasCloseToBall = false;
            }
        }

        // Obstacles are penalized purely through the sensors the physical robot
        // has. Unity's collision event has no hardware counterpart, so contact
        // stays a TensorBoard metric (see OnCollisionEnter).
        float sonar = virtualSensors != null ? virtualSensors.UltrasonicNormalized : 1f;
        bool sideIrActive = virtualSensors != null &&
                            (virtualSensors.LeftIR >= 1f || virtualSensors.RightIR >= 1f);
        if (virtualSensors != null)
        {
            AddTrackedReward(ref episodeRewardObstaclePenalty,
                -RewardTuning.SonarProximityPenalty(sonar));

            // Gradient over the worse of the two IR rays: detection at the native
            // 15 cm stays free, the cost ramps up only inside 7.5 cm. The binary
            // sideIrActive still legitimizes reversing below.
            float sideIrProximity = Mathf.Min(
                virtualSensors.LeftIRProximity, virtualSensors.RightIRProximity);
            AddTrackedReward(ref episodeRewardObstaclePenalty,
                -RewardTuning.SideIrProximityPenalty(sideIrProximity));

            if (!gripperReached && virtualSensors.GripperIR >= 1f)
            {
                gripperReached = true;
                AddTrackedReward(ref episodeRewardBonuses, RewardTuning.GripperReachedReward);
            }
        }

        // The rear has no sensors, so unjustified reversing escapes every
        // proximity signal. Reversing stays free next to an obstacle and while
        // recovering a just-lost close ball.
        bool reverseJustified = sonar < RewardTuning.ReverseEscapeClearance ||
                                sideIrActive || blindPhaseActive;
        if (gas < RewardTuning.ReverseGasThreshold && !reverseJustified)
        {
            AddTrackedReward(ref episodeRewardOtherPenalties, -RewardTuning.ReversePenalty);
        }

        // Isaac-Lab-style quadratic action-rate cost, camera servo included:
        // jerky command jumps wear out real motors and gears.
        Vector3 action = new Vector3(gas, steer, cameraTurnCommand);
        AddTrackedReward(ref episodeRewardActionRatePenalty,
            -RewardTuning.ActionRatePenalty * (action - previousAction).sqrMagnitude);
        previousAction = action;

        AddTrackedReward(ref episodeRewardOtherPenalties, -RewardTuning.DecisionStepPenalty);

        if (gripperController != null && gripperController.HasBall)
        {
            // Manual play must retain the ball until R is pressed. Training still ends the
            // episode as soon as the capture succeeds.
            if (!IsManualControl())
            {
                AddTrackedReward(ref episodeRewardTerminal, successReward);
                SuccessfulEpisodeCount++;
                ReportEpisode(true);
                EndEpisode();
            }

            return;
        }

        if ((transform.position - robotStartPosition).sqrMagnitude >
            Mathf.Pow(evaluationBoundaryRadiusOverride > 0f
                ? evaluationBoundaryRadiusOverride
                : arenaRadius, 2f) ||
            transform.position.y < robotStartPosition.y - 1f)
        {
            AddTrackedReward(ref episodeRewardTerminal, -RewardTuning.OutOfArenaPenalty);
            ReportEpisode(false);
            EndEpisode();
            return;
        }

        if (IsManualControl())
        {
            return;
        }

        if (UpdateStuckDetection(gas, steer))
        {
            episodeEndedStuck = true;
            AddTrackedReward(ref episodeRewardTerminal, -RewardTuning.StuckPenalty);
            ReportEpisode(false);
            EndEpisode();
            return;
        }

        if (Time.time - episodeStartTime > RewardTuning.EpisodeTimeLimitSeconds)
        {
            AddTrackedReward(ref episodeRewardTerminal, -RewardTuning.TimeoutPenalty);
            ReportEpisode(false);
            EndEpisode();
        }
    }

    // Commanding movement for a whole window while barely displacing (wall push,
    // spinning, tight circling) ends the episode - one decisive terminal instead
    // of the per-step spin/stuck micro-penalties of the previous reward system.
    // "Commanding" is judged on the SIGNED average of the window, so zero-mean
    // exploration dither does not read as a command (see RewardTuning notes).
    private bool UpdateStuckDetection(float gas, float steer)
    {
        stuckGasSum += gas;
        stuckSteerSum += steer;
        stuckSampleCount++;

        if (Time.time - stuckWindowStartTime < RewardTuning.StuckWindowSeconds)
        {
            return false;
        }

        float averageGas = stuckGasSum / stuckSampleCount;
        float averageSteer = stuckSteerSum / stuckSampleCount;
        bool commandingMovement =
            Mathf.Abs(averageGas) > RewardTuning.StuckCommandThreshold ||
            Mathf.Abs(averageSteer) > RewardTuning.StuckCommandThreshold;
        float displacement = Vector3.ProjectOnPlane(
            transform.position - stuckWindowStartPosition, Vector3.up).magnitude;
        float maxSpeed = trackController != null ? trackController.MaxLinearSpeed : 0.25f;
        float minimumProgress = RewardTuning.StuckMinProgressFraction *
                                maxSpeed * RewardTuning.StuckWindowSeconds;
        ResetStuckWindow();
        return commandingMovement && displacement < minimumProgress;
    }

    private void ResetStuckWindow()
    {
        stuckWindowStartTime = Time.time;
        stuckWindowStartPosition = transform.position;
        stuckGasSum = 0f;
        stuckSteerSum = 0f;
        stuckSampleCount = 0;
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
        stats.Add("Robot/StuckRate", episodeEndedStuck ? 1f : 0f);
        if (usingSuccessGatedCurriculum)
        {
            SuccessGatedCurriculum.ReportEpisode(success, episodeCollisionCount);
        }

        // Reward decomposition. Total must track Environment/Cumulative Reward;
        // a divergence means some code path bypassed AddTrackedReward.
        stats.Add("Robot/Reward/Total",
            episodeRewardApproach + episodeRewardBonuses + episodeRewardObstaclePenalty +
            episodeRewardActionRatePenalty + episodeRewardOtherPenalties + episodeRewardTerminal +
            episodeRewardDetour);
        stats.Add("Robot/Reward/Approach", episodeRewardApproach);
        stats.Add("Robot/Reward/Bonuses", episodeRewardBonuses);
        stats.Add("Robot/Reward/ObstaclePenalty", episodeRewardObstaclePenalty);
        stats.Add("Robot/Reward/ActionRatePenalty", episodeRewardActionRatePenalty);
        stats.Add("Robot/Reward/OtherPenalties", episodeRewardOtherPenalties);
        stats.Add("Robot/Reward/Terminal", episodeRewardTerminal);
        stats.Add("Robot/Reward/Detour", episodeRewardDetour);
        if (episodeDetourLayout)
        {
            stats.Add("Robot/DetourSuccessRate", success ? 1f : 0f);
        }
        episodeRunning = false;
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (IsObstacleOrWall(collision.collider.transform))
        {
            // Metric only. Obstacle avoidance is trained through the sonar/IR
            // penalties - the channels the physical robot actually perceives;
            // it has no collision event to learn from.
            episodeCollisionCount++;
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
