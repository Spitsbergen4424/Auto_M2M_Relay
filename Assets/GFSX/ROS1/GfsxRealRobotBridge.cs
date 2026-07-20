using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class GfsxRealRobotBridge : MonoBehaviour, IRobotPoseSource
{
    [Header("Connection")]
    [SerializeField] private string rosIpAddress = "192.168.2.154";
    [SerializeField] private int rosPort = 10000;
    [SerializeField] private bool dryRun = true;
    [SerializeField] private bool enableMotorCommands = false;

    [Header("Actuation")]
    [SerializeField, Min(0.001f)] private float maxLinearSpeed = 0.05f;
    [SerializeField, Min(0.001f)] private float maxAngularSpeed = 0.3f;
    [SerializeField, Range(1f, 60f)] private float publishRateHz = 10f;
    [SerializeField] private bool invertSteering;
    [SerializeField, Min(0.01f)] private float safetyStopDistanceMeters = 0.30f;
    [SerializeField] private bool prepareGripperOnEnable;

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

    public Vector3 RelativePositionMeters => new Vector3(poseEstimateX, poseEstimateY, poseEstimateZ);
    public float HeadingDegrees => poseEstimateHeadingDegrees;
    public float LinearSpeedMetersPerSecond => poseEstimateLinearSpeed;
    public float AngularSpeedRadiansPerSecond => poseEstimateAngularSpeed;
    public float MaxLinearSpeedMetersPerSecond => maxLinearSpeed;
    public bool HasPoseEstimate => poseEstimateInitialized;

    public string RosState => rosState;
    public bool DryRun => dryRun;
    public bool EnableMotorCommands => enableMotorCommands;
    public bool InvertSteering => invertSteering;
    public float SafetyStopDistanceMeters => safetyStopDistanceMeters;
    public float PublishRateHz => publishRateHz;
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

        if (!topicsRegistered)
        {
            RegisterTopicsOnce();
        }

        if (!IsMotorCommandAllowed())
        {
            lastPoseUpdateTime = Time.unscaledTime;
            if (emergencyStopLatched)
            {
                PublishZeroTwist();
            }

            nextPublishTime = Time.unscaledTime + 1f / Mathf.Max(1f, publishRateHz);
            return;
        }

        if (emergencyStopLatched)
        {
            lastPoseUpdateTime = Time.unscaledTime;
            PublishZeroTwist();
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
            PublishZeroTwist();
            return;
        }

        PublishLatestCommands();
    }

    private void OnDisable()
    {
        UnsubscribeBrain();
        PublishZeroTwist();
    }

    private void OnDestroy()
    {
        UnsubscribeBrain();
        PublishZeroTwist();
    }

    private void OnApplicationQuit()
    {
        PublishZeroTwist();
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

    public void PrepareGripper()
    {
        if (!IsMotorCommandAllowed())
        {
            return;
        }

        sendPrepareGripperOnce = true;
        if (Time.unscaledTime >= nextPublishTime)
        {
            PublishLatestCommands();
        }
    }

    public void EmergencyStop()
    {
        emergencyStopLatched = true;
        PublishZeroTwist();
    }

    public void ClearEmergencyStop()
    {
        emergencyStopLatched = false;
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
    }

    private void OnSensorData(QuaternionMsg message)
    {
        if (realRobotSensors == null)
        {
            return;
        }

        realRobotSensors.ApplySensorData(message.x, message.y, message.z, message.w);
        lastSensorPacketAgeSeconds = 0f;
    }

    private void OnSensorPwm(Vector3Msg message)
    {
        if (realRobotSensors == null)
        {
            return;
        }

        realRobotSensors.ApplyTrackPwm(message.x, message.y);
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
            PublishZeroTwist();
            return;
        }

        if (emergencyStopLatched)
        {
            PublishZeroTwist();
            return;
        }

        bool safetyStopped = false;
        float linearX = Mathf.Clamp(lastPpoContinuousActions.x, -1f, 1f) * maxLinearSpeed;
        float angularZ = Mathf.Clamp(lastPpoContinuousActions.y, -1f, 1f) * maxAngularSpeed;
        if (invertSteering)
        {
            angularZ = -angularZ;
        }

        if (realRobotSensors != null && realRobotSensors.IsDataFresh && linearX > 0f &&
            realRobotSensors.UltrasonicMeters < safetyStopDistanceMeters)
        {
            linearX = 0f;
            safetyStopped = true;
        }

        float publishYaw = Mathf.Clamp(lastPpoContinuousActions.z, -1f, 1f);
        PublishTwist(linearX, angularZ);
        PublishCameraPan(publishYaw);

        if (sendPrepareGripperOnce)
        {
            PublishGripperCommand(1);
            sendPrepareGripperOnce = false;
        }

        IntegratePoseEstimate(linearX, angularZ);
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
        if (realRobotSensors != null)
        {
            lastSensorPacketAgeSeconds = realRobotSensors.LastPacketAgeSeconds;
            if (!realRobotSensors.IsDataFresh)
            {
                return true;
            }
        }

        if (realYoloCamera != null)
        {
            lastYoloPacketAgeSeconds = realYoloCamera.LastPacketAgeSeconds;
            if (!realYoloCamera.IsReceivingPackets)
            {
                return true;
            }
        }

        return false;
    }

    private void UpdateDiagnostics()
    {
        if (realRobotSensors != null)
        {
            lastSensorPacketAgeSeconds = realRobotSensors.LastPacketAgeSeconds;
        }

        if (realYoloCamera != null)
        {
            lastYoloPacketAgeSeconds = realYoloCamera.LastPacketAgeSeconds;
        }
    }

    private void PublishZeroTwist()
    {
        if (!IsMotorCommandAllowed())
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
        }
        catch (Exception exception)
        {
            rosState = "Twist publish failed";
            PublishZeroTwist();
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

    private string BuildState(string state, string reason)
    {
        return string.IsNullOrWhiteSpace(reason) ? state : $"{state} ({reason})";
    }
}
