using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class ROSBridge : MonoBehaviour
{
    [Header("Activation")]
    [SerializeField] private bool realRobotMode;
    [SerializeField] private VirtualSensors sensorTarget;
    [SerializeField] private SimulatedYoloCamera cameraTarget;
    [SerializeField] private RealVision realVision;

    [Header("Command topics")]
    [SerializeField] private string velocityTopic = "/cmd_vel";
    [SerializeField] private string gripperTopic = "/cmd_gripper";
    [SerializeField] private string cameraPanTopic = "/cmd_camera_pan";

    [Header("Sensor topics")]
    [SerializeField] private string ultrasonicTopic = "/gfsx/ultrasonic";
    [SerializeField] private string leftIrTopic = "/gfsx/left_ir";
    [SerializeField] private string rightIrTopic = "/gfsx/right_ir";
    [SerializeField] private string gripperIrTopic = "/gfsx/gripper_ir";
    [SerializeField] private string ballVisibleTopic = "/gfsx/ball_visible";
    [SerializeField] private string ballHorizontalTopic = "/gfsx/ball_horizontal";
    [SerializeField] private string ballDistanceTopic = "/gfsx/ball_distance";

    [Header("Real robot limits")]
    [SerializeField, Min(0.01f)] private float maxLinearSpeed = 0.25f;
    [SerializeField, Min(0.01f)] private float maxAngularSpeed = 1f;
    [SerializeField, Min(0.01f)] private float ultrasonicMaxDistance = 2f;
    [SerializeField, Min(0.01f)] private float frontSafetyStopDistance = 0.50f;
    [SerializeField, Min(0.01f)] private float cameraMaxDistance = 2f;
    [SerializeField, Min(0.01f)] private float cameraYawLimitDegrees = 70f;
    [SerializeField, Min(0.000001f)] private float ultrasonicInputToMeters = 1f;
    [SerializeField, Min(0.000001f)] private float ballDistanceInputToMeters = 1f;
    [SerializeField, Range(0.1f, 1f)] private float emaAlpha = 0.8f;
    [SerializeField, Range(0f, 0.1f)] private float commandDeadzone = 0.01f;

    [Header("Hardware conventions")]
    [SerializeField] private bool invertLeftIr;
    [SerializeField] private bool invertRightIr;
    [SerializeField] private bool invertGripperIr;
    [SerializeField] private int rosPrepareGripperCommand = 1;
    [SerializeField] private int rosCloseAndLiftGripperCommand = 2;
    [SerializeField] private int rosReleaseGripperCommand = 4;

    [Header("Fail-safe")]
    [SerializeField, Min(0.1f)] private float commandWatchdogSeconds = 0.5f;
    [SerializeField, Min(0.1f)] private float sensorWatchdogSeconds = 0.75f;

    private ROSConnection ros;
    private float smoothGas;
    private float smoothSteering;
    private float lastCommandTime;
    private float lastSensorTime;
    private bool commandReceived;
    private bool sensorReceived;
    private bool commandWatchdogTriggered;
    private bool sensorWatchdogTriggered;
    private bool ballVisible;
    private float ballHorizontal;
    private float ballDistanceNormalized = 1f;
    private bool rosInitialized;
    private bool realGripperPrepared;
    private bool realGripperIrActive;
    private float latestUltrasonicMeters = float.PositiveInfinity;

    public bool RealRobotMode => realRobotMode;
    public bool SensorsFresh => sensorReceived &&
                                Time.realtimeSinceStartup - lastSensorTime <= sensorWatchdogSeconds;

    private void Awake()
    {
        sensorTarget ??= GetComponent<VirtualSensors>();
        cameraTarget ??= GetComponentInChildren<SimulatedYoloCamera>(true);
        realVision ??= GetComponent<RealVision>();
    }

    private void Start()
    {
        if (!realRobotMode)
        {
            return;
        }

        InitializeRos();
    }

    private void InitializeRos()
    {
        if (!realRobotMode || rosInitialized)
        {
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(velocityTopic);
        ros.RegisterPublisher<Int32Msg>(gripperTopic);
        ros.RegisterPublisher<Float32Msg>(cameraPanTopic);

        ros.Subscribe<Float32Msg>(ultrasonicTopic, ReceiveUltrasonic);
        ros.Subscribe<Int32Msg>(leftIrTopic, message => ReceiveIr(message, SensorChannel.Left));
        ros.Subscribe<Int32Msg>(rightIrTopic, message => ReceiveIr(message, SensorChannel.Right));
        ros.Subscribe<Int32Msg>(gripperIrTopic, message => ReceiveIr(message, SensorChannel.Gripper));
        // P7 YOLO sends vision directly to Unity over UDP. Keep the ROS topics
        // as a fallback for deployments that provide ROS-native YOLO data.
        if (realVision == null)
        {
            ros.Subscribe<Int32Msg>(ballVisibleTopic, ReceiveBallVisible);
            ros.Subscribe<Float32Msg>(ballHorizontalTopic, ReceiveBallHorizontal);
            ros.Subscribe<Float32Msg>(ballDistanceTopic, ReceiveBallDistance);
        }

        sensorTarget?.SetExternalMode(true);
        cameraTarget?.SetExternalMode(true);
        realVision?.SetExternalMode(true);
        lastCommandTime = Time.realtimeSinceStartup;
        lastSensorTime = Time.realtimeSinceStartup;
        PublishEmergencyStop("ROS bridge started");
        rosInitialized = true;
    }

    private void Update()
    {
        if (!realRobotMode || ros == null)
        {
            return;
        }

        float now = Time.realtimeSinceStartup;
        if (commandReceived && !commandWatchdogTriggered && now - lastCommandTime > commandWatchdogSeconds)
        {
            commandWatchdogTriggered = true;
            PublishEmergencyStop($"No AI command for {commandWatchdogSeconds:F2} s");
        }

        if (!sensorWatchdogTriggered && now - lastSensorTime > sensorWatchdogSeconds)
        {
            sensorWatchdogTriggered = true;
            sensorTarget?.MarkExternalDataStale();
            cameraTarget?.MarkExternalDataStale();
            PublishEmergencyStop($"No real sensor data for {sensorWatchdogSeconds:F2} s");
        }
    }

    public void PublishCommand(float gas, float steering)
    {
        if (!CanPublish())
        {
            return;
        }

        // Never let a new inference action override the sensor watchdog stop.
        // Motion is unlocked only after at least one fresh ROS sensor message.
        if (!SensorsFresh)
        {
            smoothGas = 0f;
            smoothSteering = 0f;
            PublishTwist(0f, 0f);
            return;
        }

        gas = Mathf.Clamp(gas, -1f, 1f);
        steering = Mathf.Clamp(steering, -1f, 1f);
        bool hardStop = Mathf.Abs(gas) <= commandDeadzone && Mathf.Abs(steering) <= commandDeadzone;
        if (hardStop)
        {
            smoothGas = 0f;
            smoothSteering = 0f;
        }
        else
        {
            smoothGas = emaAlpha * gas + (1f - emaAlpha) * smoothGas;
            smoothSteering = emaAlpha * steering + (1f - emaAlpha) * smoothSteering;
        }

        float linear = smoothGas * maxLinearSpeed;
        if (linear > 0f && latestUltrasonicMeters <= frontSafetyStopDistance)
        {
            // Keep turning available: it is the safe way to leave an obstacle.
            linear = 0f;
        }

        PublishTwist(linear, smoothSteering * maxAngularSpeed);
        commandReceived = true;
        commandWatchdogTriggered = false;
        lastCommandTime = Time.realtimeSinceStartup;
    }

    public void PublishGripperCmd(int command)
    {
        if (!CanPublish())
        {
            return;
        }

        // Unity actions: 1 = try to grab, 2 = release.
        // Hardware protocol: 1 = lower/open, 2 = close/lift, 4 = open only.
        if (command == 1)
        {
            if (realGripperIrActive)
            {
                PublishPhysicalGripperCommand(rosCloseAndLiftGripperCommand);
            }
            else
            {
                realGripperPrepared = true;
                PublishPhysicalGripperCommand(rosPrepareGripperCommand);
            }
        }
        else if (command == 2)
        {
            realGripperPrepared = false;
            PublishPhysicalGripperCommand(rosReleaseGripperCommand);
        }
    }

    public void PublishCameraCmd(float yaw)
    {
        if (CanPublish())
        {
            float normalizedYaw = cameraYawLimitDegrees > 0f
                ? Mathf.Clamp(yaw / cameraYawLimitDegrees, -1f, 1f)
                : 0f;
            ros.Publish(cameraPanTopic, new Float32Msg(normalizedYaw));
        }
    }

    public void SetRealRobotMode(bool enabled)
    {
        realRobotMode = enabled;
        sensorTarget?.SetExternalMode(enabled);
        cameraTarget?.SetExternalMode(enabled);
        realVision?.SetExternalMode(enabled);
        if (!enabled && ros != null)
        {
            PublishEmergencyStop("Real robot mode disabled");
        }
        else if (enabled)
        {
            InitializeRos();
        }
    }

    private bool CanPublish()
    {
        return realRobotMode && ros != null;
    }

    private void PublishEmergencyStop(string reason)
    {
        smoothGas = 0f;
        smoothSteering = 0f;
        if (ros != null)
        {
            PublishTwist(0f, 0f);
        }

        Debug.LogWarning($"ROSBridge fail-safe stop: {reason}", this);
    }

    private void PublishTwist(float linear, float angular)
    {
        var command = new TwistMsg();
        command.linear.x = linear;
        command.angular.z = angular;
        ros.Publish(velocityTopic, command);
    }

    private void ReceiveUltrasonic(Float32Msg message)
    {
        MarkSensorReceived();
        latestUltrasonicMeters = Mathf.Max(0f, message.data * ultrasonicInputToMeters);
        sensorTarget?.SetExternalUltrasonicMeters(latestUltrasonicMeters, ultrasonicMaxDistance);
    }

    private enum SensorChannel
    {
        Left,
        Right,
        Gripper
    }

    private void ReceiveIr(Int32Msg message, SensorChannel channel)
    {
        MarkSensorReceived();
        bool active = message.data != 0;
        if (sensorTarget == null)
        {
            return;
        }

        switch (channel)
        {
            case SensorChannel.Left:
                sensorTarget.SetExternalLeftIr(active ^ invertLeftIr ? 1f : 0f);
                break;
            case SensorChannel.Right:
                sensorTarget.SetExternalRightIr(active ^ invertRightIr ? 1f : 0f);
                break;
            case SensorChannel.Gripper:
                sensorTarget.SetExternalGripperIr(active ^ invertGripperIr ? 1f : 0f);
                realGripperIrActive = active ^ invertGripperIr;
                if (realGripperPrepared && realGripperIrActive)
                {
                    realGripperPrepared = false;
                    PublishPhysicalGripperCommand(rosCloseAndLiftGripperCommand);
                }
                break;
        }
    }

    private void ReceiveBallVisible(Int32Msg message)
    {
        MarkSensorReceived();
        ballVisible = message.data != 0;
        PushExternalBallDetection();
    }

    private void ReceiveBallHorizontal(Float32Msg message)
    {
        MarkSensorReceived();
        ballHorizontal = Mathf.Clamp(message.data, -1f, 1f);
        PushExternalBallDetection();
    }

    private void ReceiveBallDistance(Float32Msg message)
    {
        MarkSensorReceived();
        ballDistanceNormalized = Mathf.Clamp01(
            Mathf.Max(0f, message.data * ballDistanceInputToMeters) / cameraMaxDistance);
        PushExternalBallDetection();
    }

    private void PushExternalBallDetection()
    {
        cameraTarget?.SetExternalDetection(ballVisible, ballHorizontal, ballDistanceNormalized);
    }

    private void MarkSensorReceived()
    {
        sensorReceived = true;
        sensorWatchdogTriggered = false;
        lastSensorTime = Time.realtimeSinceStartup;
    }

    private void PublishPhysicalGripperCommand(int command)
    {
        if (CanPublish())
        {
            ros.Publish(gripperTopic, new Int32Msg(command));
        }
    }

    private void OnDisable()
    {
        if (realRobotMode && ros != null)
        {
            PublishEmergencyStop("ROS bridge disabled");
        }
    }

    private void OnApplicationQuit()
    {
        if (realRobotMode && ros != null)
        {
            PublishEmergencyStop("Unity application quitting");
        }
    }
}
