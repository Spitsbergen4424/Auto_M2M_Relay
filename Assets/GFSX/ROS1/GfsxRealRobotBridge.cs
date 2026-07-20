using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class GfsxRealRobotBridge : MonoBehaviour, IRobotPoseSource, IRobotCaptureSource
{
    [Header("Connection")]
    [SerializeField] private string rosIpAddress = "192.168.2.154";
    [SerializeField] private int rosPort = 10000;
    [SerializeField] private bool dryRun = true;
    [SerializeField] private bool enableMotorCommands = false;

    [Header("Actuation")]
    [SerializeField, Min(0.001f)] private float maxLinearSpeed = 0.25f;
    [SerializeField, Min(0.001f)] private float maxAngularSpeed = 0.9f;
    [SerializeField, Range(0f, 0.9f)] private float actionDeadband = 0.15f;
    [SerializeField, Min(0f)] private float minimumEffectiveLinearSpeed = 0.175f;
    [SerializeField, Min(0f)] private float minimumEffectiveAngularSpeed = 0.70f;
    [SerializeField, Range(1f, 60f)] private float publishRateHz = 10f;
    [SerializeField] private bool invertSteering;
    [SerializeField, Min(0.01f)] private float safetyStopDistanceMeters = 0.30f;
    [SerializeField] private bool prepareGripperOnEnable;

    [Header("Hardware drive model")]
    [SerializeField, Min(0.01f)] private float robotTurnK = 0.25f;
    [SerializeField, Min(1f)] private float pwmPerMeterPerSecond = 200f;
    [SerializeField, Min(0f)] private float motorDeadZonePwm = 10f;
    [SerializeField, Min(0f)] private float minimumMotorPwm = 35f;

    [Header("Freshness and capture")]
    [SerializeField, Min(0.05f)] private float actionTimeoutSeconds = 0.5f;
    [SerializeField, Min(0f)] private float gripperCaptureConfirmSeconds = 0.15f;
    [SerializeField] private bool stopAfterCapture = true;

    [Header("Topics")]
    [SerializeField] private string cmdVelTopic = "/cmd_vel";
    [SerializeField] private string cmdCameraPanTopic = "/cmd_camera_pan";
    [SerializeField] private string cmdGripperTopic = "/cmd_gripper";
    [SerializeField] private string sensorDataTopic = "/sensor/data";
    [SerializeField] private string sensorPwmTopic = "/sensor/pwm";
    [SerializeField] private string sensorGripperIrTopic = "/sensor/gripper_ir";

    [Header("References")]
    [SerializeField] private RobotBrain robotBrain;
    [SerializeField] private RealRobotSensors realRobotSensors;
    [SerializeField] private RealYoloCamera realYoloCamera;

    [Header("Diagnostics")]
    [SerializeField] private string rosState = "Idle";
    [SerializeField] private float lastSensorPacketAgeSeconds = float.PositiveInfinity;
    [SerializeField] private float lastYoloPacketAgeSeconds = float.PositiveInfinity;
    [SerializeField] private Vector3 lastPpoContinuousActions;
    [SerializeField] private int lastPpoGripperCommand;
    [SerializeField] private float lastSentLinearX;
    [SerializeField] private float lastSentAngularZ;
    [SerializeField] private float lastSentCameraPan;
    [SerializeField] private int lastSentGripperCommand;
    [SerializeField] private bool emergencyStopLatched;
    [SerializeField] private bool ballCaptured;
    [SerializeField] private float lastActionAgeSeconds = float.PositiveInfinity;
    [SerializeField] private float poseEstimateX;
    [SerializeField] private float poseEstimateY;
    [SerializeField] private float poseEstimateZ;
    [SerializeField] private float poseEstimateHeadingDegrees;
    [SerializeField] private float poseEstimateLinearSpeed;
    [SerializeField] private float poseEstimateAngularSpeed;

    private ROSConnection rosConnection;
    private bool topicsRegistered;
    private bool actionSubscribed;
    private bool sendPrepareGripperOnce;
    private float nextPublishTime;
    private float lastPoseUpdateTime;
    private bool poseEstimateInitialized;
    private float lastActionTime = float.NegativeInfinity;
    private float gripperIrActiveSince = float.NegativeInfinity;
    private bool motorControlWasAllowed;
    private bool motorCommandWasEverPublished;

    public Vector3 RelativePositionMeters => new Vector3(poseEstimateX, poseEstimateY, poseEstimateZ);
    public float HeadingDegrees => poseEstimateHeadingDegrees;
    public float LinearSpeedMetersPerSecond => poseEstimateLinearSpeed;
    public float AngularSpeedRadiansPerSecond => poseEstimateAngularSpeed;
    public float MaxLinearSpeedMetersPerSecond => maxLinearSpeed;
    public bool HasPoseEstimate => poseEstimateInitialized;
    public bool HasCapturedBall => ballCaptured;

    public string RosState => rosState;
    public bool DryRun => dryRun;
    public bool EnableMotorCommands => enableMotorCommands;
    public bool InvertSteering => invertSteering;
    public float SafetyStopDistanceMeters => safetyStopDistanceMeters;
    public float PublishRateHz => publishRateHz;
    public float MaxAngularSpeedRadiansPerSecond => maxAngularSpeed;
    public float ActionDeadband => actionDeadband;
    public bool StopAfterCapture => stopAfterCapture;
    public string RosIpAddress => rosIpAddress;
    public int RosPort => rosPort;
    public float LastSensorPacketAge => lastSensorPacketAgeSeconds;
    public float LastYoloPacketAge => lastYoloPacketAgeSeconds;
    public Vector3 LastPpoContinuousActions => lastPpoContinuousActions;
    public int LastPpoGripperCommand => lastPpoGripperCommand;

    public void ConfigureRealMode(string ipAddress, int port, bool dryRunEnabled, bool motorCommandsEnabled,
        float linearSpeed, float angularSpeed, float publishHzValue, bool invertSteeringValue,
        float safetyStopDistance, bool prepareGripper)
    {
        rosIpAddress = ipAddress;
        rosPort = port;
        dryRun = dryRunEnabled;
        enableMotorCommands = motorCommandsEnabled;
        maxLinearSpeed = Mathf.Max(0.001f, linearSpeed);
        maxAngularSpeed = Mathf.Max(0.001f, angularSpeed);
        publishRateHz = Mathf.Clamp(publishHzValue, 1f, 60f);
        invertSteering = invertSteeringValue;
        safetyStopDistanceMeters = Mathf.Max(0.01f, safetyStopDistance);
        prepareGripperOnEnable = prepareGripper;
        emergencyStopLatched = false;
        sendPrepareGripperOnce = false;
        lastSentLinearX = 0f;
        lastSentAngularZ = 0f;
        lastSentCameraPan = 0f;
        lastSentGripperCommand = 0;
        lastActionTime = float.NegativeInfinity;
        ballCaptured = false;
        gripperIrActiveSince = float.NegativeInfinity;
        motorControlWasAllowed = false;
        motorCommandWasEverPublished = false;
        PrepareConnection();
        ResetPoseEstimate();
        rosState = dryRun ? "DryRun" : "Idle";
    }

    private void Awake()
    {
        CacheReferences();
        PrepareConnection();
        ResetPoseEstimate();
    }

    private void OnEnable()
    {
        CacheReferences();
        SubscribeBrain();
        PrepareConnection();
    }

    private void Start()
    {
        RegisterTopicsOnce();
        ConnectOnce();
        UpdateDiagnostics();

        if (prepareGripperOnEnable)
        {
            sendPrepareGripperOnce = true;
        }
    }

    private void Update()
    {
        UpdateDiagnostics();
        UpdateCaptureLatch();

        if (!topicsRegistered)
        {
            RegisterTopicsOnce();
        }

        bool motorAllowedNow = IsMotorCommandAllowed();
        if (!motorAllowedNow)
        {
            if (motorControlWasAllowed)
            {
                PublishZeroTwistAfterArming();
            }

            motorControlWasAllowed = false;
            lastPoseUpdateTime = Time.unscaledTime;
            nextPublishTime = Time.unscaledTime + 1f / Mathf.Max(1f, publishRateHz);
            return;
        }

        motorControlWasAllowed = true;

        if (emergencyStopLatched || (stopAfterCapture && ballCaptured))
        {
            lastPoseUpdateTime = Time.unscaledTime;
            PublishZeroTwistAfterArming();
            nextPublishTime = Time.unscaledTime + 1f / Mathf.Max(1f, publishRateHz);
            return;
        }

        if (Time.unscaledTime < nextPublishTime)
        {
            return;
        }

        nextPublishTime = Time.unscaledTime + 1f / Mathf.Max(1f, publishRateHz);

        if (ShouldStopForStaleData())
        {
            rosState = BuildState("Safety stop", "stale data");
            PublishZeroTwistAfterArming();
            return;
        }

        PublishLatestCommands();
    }

    private void OnDisable()
    {
        UnsubscribeBrain();
        PublishZeroTwistAfterArming();
    }

    private void OnDestroy()
    {
        UnsubscribeBrain();
        PublishZeroTwistAfterArming();
    }

    private void OnApplicationQuit()
    {
        PublishZeroTwistAfterArming();
    }

    public void ResetPoseEstimate()
    {
        poseEstimateX = 0f;
        poseEstimateY = 0f;
        poseEstimateZ = 0f;
        poseEstimateHeadingDegrees = 0f;
        poseEstimateLinearSpeed = 0f;
        poseEstimateAngularSpeed = 0f;
        poseEstimateInitialized = true;
        lastPoseUpdateTime = Time.unscaledTime;
    }

    [ContextMenu("Reset Pose Estimate")]
    public void ResetPoseEstimateFromInspector()
    {
        ResetPoseEstimate();
    }

    [ContextMenu("Prepare Gripper")]
    public void PrepareGripper()
    {
        if (!IsMotorCommandAllowed() || ShouldStopForStaleData() || emergencyStopLatched || ballCaptured)
        {
            return;
        }

        PublishGripperCommand(1);
        sendPrepareGripperOnce = false;
    }

    [ContextMenu("Emergency Stop")]
    public void EmergencyStop()
    {
        emergencyStopLatched = true;
        PublishZeroTwistAfterArming();
    }

    [ContextMenu("Clear Emergency Stop")]
    public void ClearEmergencyStop()
    {
        emergencyStopLatched = false;
    }

    [ContextMenu("Reset Captured Ball State")]
    public void ResetCapturedState()
    {
        ballCaptured = false;
        gripperIrActiveSince = float.NegativeInfinity;
    }

    private void CacheReferences()
    {
        if (robotBrain == null)
        {
            robotBrain = GetComponent<RobotBrain>();
        }

        if (realRobotSensors == null)
        {
            realRobotSensors = GetComponent<RealRobotSensors>();
        }

        if (realYoloCamera == null)
        {
            realYoloCamera = GetComponentInChildren<RealYoloCamera>(true);
        }
    }

    private void PrepareConnection()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosConnection.RosIPAddress = rosIpAddress;
        rosConnection.RosPort = rosPort;
        rosConnection.ConnectOnStart = false;
        rosState = dryRun ? "DryRun" : "Idle";
    }

    private void ConnectOnce()
    {
        try
        {
            rosConnection.Connect(rosIpAddress, rosPort);
            rosState = dryRun ? "Connected (dry-run)" : "Connected";
        }
        catch (Exception exception)
        {
            rosState = "Connect failed";
            Debug.LogError($"GFS-X real robot bridge connection failed: {exception.Message}", this);
        }
    }

    private void RegisterTopicsOnce()
    {
        if (topicsRegistered || rosConnection == null)
        {
            return;
        }

        rosConnection.Subscribe<QuaternionMsg>(sensorDataTopic, OnSensorData);
        rosConnection.Subscribe<Vector3Msg>(sensorPwmTopic, OnSensorPwm);
        rosConnection.Subscribe<Int32Msg>(sensorGripperIrTopic, OnSensorGripperIr);

        rosConnection.RegisterPublisher<TwistMsg>(cmdVelTopic);
        rosConnection.RegisterPublisher<Float32Msg>(cmdCameraPanTopic);
        rosConnection.RegisterPublisher<Int32Msg>(cmdGripperTopic);

        topicsRegistered = true;
    }

    private void SubscribeBrain()
    {
        if (robotBrain == null || actionSubscribed)
        {
            return;
        }

        robotBrain.ActionComputed += OnRobotActionComputed;
        robotBrain.SetExternalActuationEnabled(true);
        actionSubscribed = true;
    }

    private void UnsubscribeBrain()
    {
        if (robotBrain != null && actionSubscribed)
        {
            robotBrain.ActionComputed -= OnRobotActionComputed;
            robotBrain.SetExternalActuationEnabled(false);
        }

        actionSubscribed = false;
    }

    private void OnRobotActionComputed(RobotActionCommand command)
    {
        lastPpoContinuousActions = new Vector3(command.Gas, command.Steer, command.CameraPan);
        lastPpoGripperCommand = command.GripperCommand;
        lastActionTime = Time.unscaledTime;
    }

    private void OnSensorData(QuaternionMsg message)
    {
        if (realRobotSensors == null)
        {
            return;
        }

        realRobotSensors.ApplySensorData((float)message.x, (float)message.y, (float)message.z, (float)message.w);
        lastSensorPacketAgeSeconds = 0f;
    }

    private void OnSensorPwm(Vector3Msg message)
    {
        if (realRobotSensors == null)
        {
            return;
        }

        realRobotSensors.ApplyTrackPwm((float)message.x, (float)message.y);
        lastSensorPacketAgeSeconds = 0f;
    }

    private void OnSensorGripperIr(Int32Msg message)
    {
        if (realRobotSensors == null)
        {
            return;
        }

        realRobotSensors.ApplyGripperIr(message.data);
        lastSensorPacketAgeSeconds = 0f;
    }

    private void PublishLatestCommands()
    {
        if (!IsMotorCommandAllowed())
        {
            PublishZeroTwistAfterArming();
            return;
        }

        if (emergencyStopLatched)
        {
            PublishZeroTwistAfterArming();
            return;
        }

        bool safetyStopped = false;
        float linearX = MapEffectiveCommand(lastPpoContinuousActions.x, maxLinearSpeed,
            minimumEffectiveLinearSpeed, actionDeadband);
        float angularZ = MapEffectiveCommand(lastPpoContinuousActions.y, maxAngularSpeed,
            minimumEffectiveAngularSpeed, actionDeadband);
        if (invertSteering)
        {
            angularZ = -angularZ;
        }

        if (realRobotSensors != null && realRobotSensors.IsSensorDataFresh && linearX > 0f &&
            realRobotSensors.UltrasonicMeters < safetyStopDistanceMeters)
        {
            linearX = 0f;
            safetyStopped = true;
        }

        float publishYaw = robotBrain != null ? robotBrain.NormalizedCameraYaw : 0f;
        PublishTwist(linearX, angularZ);
        PublishCameraPan(publishYaw);

        if (sendPrepareGripperOnce)
        {
            PublishGripperCommand(1);
            sendPrepareGripperOnce = false;
        }

        EstimateActualChassisMotion(linearX, angularZ, out float estimatedLinear, out float estimatedAngular);
        IntegratePoseEstimate(estimatedLinear, estimatedAngular);
        rosState = safetyStopped
            ? BuildState("Safety stop", realRobotSensors != null
                ? $"ultrasonic={realRobotSensors.UltrasonicMeters:F2}m"
                : "ultrasonic stale")
            : BuildState("Active", dryRun ? "dry-run" : "motor publish");
    }

    private bool IsMotorCommandAllowed()
    {
        return !dryRun && enableMotorCommands && rosConnection != null && topicsRegistered;
    }

    private bool ShouldStopForStaleData()
    {
        if (realRobotSensors == null || !realRobotSensors.IsSensorDataFresh)
        {
            return true;
        }

        if (realYoloCamera == null || !realYoloCamera.IsReceivingPackets)
        {
            return true;
        }

        lastActionAgeSeconds = PacketAge(lastActionTime);
        if (lastActionAgeSeconds > actionTimeoutSeconds)
        {
            return true;
        }

        return false;
    }

    private void UpdateDiagnostics()
    {
        if (realRobotSensors != null)
        {
            lastSensorPacketAgeSeconds = realRobotSensors.LastSensorDataPacketAgeSeconds;
        }

        if (realYoloCamera != null)
        {
            lastYoloPacketAgeSeconds = realYoloCamera.LastPacketAgeSeconds;
        }

        lastActionAgeSeconds = PacketAge(lastActionTime);
    }

    private void PublishZeroTwistAfterArming()
    {
        if (!motorCommandWasEverPublished || rosConnection == null || !topicsRegistered)
        {
            return;
        }

        try
        {
            rosConnection.Publish(cmdVelTopic,
                new TwistMsg(new Vector3Msg(0f, 0f, 0f), new Vector3Msg(0f, 0f, 0f)));
            lastSentLinearX = 0f;
            lastSentAngularZ = 0f;
            poseEstimateLinearSpeed = 0f;
            poseEstimateAngularSpeed = 0f;
            lastPoseUpdateTime = Time.unscaledTime;
        }
        catch (Exception exception)
        {
            rosState = "Stop publish failed";
            Debug.LogWarning($"GFS-X real robot stop failed: {exception.Message}", this);
        }
    }

    private void PublishTwist(float linearX, float angularZ)
    {
        try
        {
            rosConnection.Publish(cmdVelTopic,
                new TwistMsg(new Vector3Msg(linearX, 0f, 0f), new Vector3Msg(0f, 0f, angularZ)));
            lastSentLinearX = linearX;
            lastSentAngularZ = angularZ;
            motorCommandWasEverPublished = true;
        }
        catch (Exception exception)
        {
            rosState = "Twist publish failed";
            PublishZeroTwistAfterArming();
            Debug.LogWarning($"GFS-X real robot twist publish failed: {exception.Message}", this);
        }
    }

    private void PublishCameraPan(float cameraPan)
    {
        try
        {
            rosConnection.Publish(cmdCameraPanTopic, new Float32Msg(cameraPan));
            lastSentCameraPan = cameraPan;
        }
        catch (Exception exception)
        {
            rosState = "Camera publish failed";
            Debug.LogWarning($"GFS-X real robot camera pan publish failed: {exception.Message}", this);
        }
    }

    private void PublishGripperCommand(int command)
    {
        if (command <= 0)
        {
            return;
        }

        try
        {
            rosConnection.Publish(cmdGripperTopic, new Int32Msg(command));
            lastSentGripperCommand = command;
        }
        catch (Exception exception)
        {
            rosState = "Gripper publish failed";
            Debug.LogWarning($"GFS-X real robot gripper publish failed: {exception.Message}", this);
        }
    }

    private void IntegratePoseEstimate(float linearX, float angularZ)
    {
        if (!poseEstimateInitialized || !IsMotorCommandAllowed())
        {
            lastPoseUpdateTime = Time.unscaledTime;
            poseEstimateLinearSpeed = 0f;
            poseEstimateAngularSpeed = 0f;
            return;
        }

        float now = Time.unscaledTime;
        float deltaTime = Mathf.Max(0f, now - lastPoseUpdateTime);
        lastPoseUpdateTime = now;
        poseEstimateLinearSpeed = linearX;
        poseEstimateAngularSpeed = angularZ;

        float headingRadians = poseEstimateHeadingDegrees * Mathf.Deg2Rad;
        headingRadians += angularZ * deltaTime;
        poseEstimateHeadingDegrees = headingRadians * Mathf.Rad2Deg;

        Quaternion headingRotation = Quaternion.AngleAxis(poseEstimateHeadingDegrees, Vector3.up);
        Vector3 direction = headingRotation * transform.right;
        Vector3 planarStep = direction.normalized * (linearX * deltaTime);
        poseEstimateX += planarStep.x;
        poseEstimateY += planarStep.y;
        poseEstimateZ += planarStep.z;
    }

    private void UpdateCaptureLatch()
    {
        if (ballCaptured || realRobotSensors == null || !realRobotSensors.IsGripperSignalFresh ||
            realRobotSensors.GripperIr < 0.5f)
        {
            if (!ballCaptured)
            {
                gripperIrActiveSince = float.NegativeInfinity;
            }
            return;
        }

        if (float.IsNegativeInfinity(gripperIrActiveSince))
        {
            gripperIrActiveSince = Time.unscaledTime;
            return;
        }

        if (Time.unscaledTime - gripperIrActiveSince >= gripperCaptureConfirmSeconds)
        {
            ballCaptured = true;
            rosState = "Captured ball";
            PublishZeroTwistAfterArming();
        }
    }

    public static float MapEffectiveCommand(float normalizedAction, float maximumMagnitude,
        float minimumEffectiveMagnitude, float deadband)
    {
        float magnitude = Mathf.Abs(Mathf.Clamp(normalizedAction, -1f, 1f));
        float clampedDeadband = Mathf.Clamp(deadband, 0f, 0.99f);
        if (magnitude <= clampedDeadband)
        {
            return 0f;
        }

        float normalized = Mathf.InverseLerp(clampedDeadband, 1f, magnitude);
        float minimum = Mathf.Min(Mathf.Max(0f, minimumEffectiveMagnitude), maximumMagnitude);
        float mapped = Mathf.Lerp(minimum, maximumMagnitude, normalized);
        return Mathf.Sign(normalizedAction) * mapped;
    }

    private void EstimateActualChassisMotion(float linearX, float angularZ,
        out float estimatedLinear, out float estimatedAngular)
    {
        float turnK = Mathf.Max(0.01f, robotTurnK);
        float left = ShapeWheelVelocity(linearX + angularZ * turnK);
        float right = ShapeWheelVelocity(linearX - angularZ * turnK);
        estimatedLinear = (left + right) * 0.5f;
        estimatedAngular = (left - right) / (2f * turnK);
    }

    private float ShapeWheelVelocity(float requestedVelocity)
    {
        float pwmFactor = Mathf.Max(1f, pwmPerMeterPerSecond);
        float rawPwm = Mathf.Clamp(requestedVelocity * pwmFactor, -100f, 100f);
        float magnitude = Mathf.Abs(rawPwm);
        if (magnitude < motorDeadZonePwm)
        {
            return 0f;
        }

        float shapedPwm = Mathf.Max(magnitude, minimumMotorPwm);
        return Mathf.Sign(rawPwm) * shapedPwm / pwmFactor;
    }

    private static float PacketAge(float timestamp)
    {
        return float.IsNegativeInfinity(timestamp)
            ? float.PositiveInfinity
            : Time.unscaledTime - timestamp;
    }

    private string BuildState(string state, string reason)
    {
        return string.IsNullOrWhiteSpace(reason) ? state : $"{state} ({reason})";
    }
}
