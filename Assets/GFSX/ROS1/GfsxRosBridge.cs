using System;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(GfsxDriveAdapter))]
public sealed class GfsxRosBridge : MonoBehaviour
{
    [Header("Connection")]
    [SerializeField] private string rosIpAddress = "127.0.0.1";
    [SerializeField] private int rosPort = 10000;
    [SerializeField] private bool connectOnStart = true;

    [Header("Adapters")]
    [SerializeField] private GfsxDriveAdapter driveAdapter;
    [SerializeField] private GfsxGripperAdapter gripperAdapter;
    [SerializeField] private GfsxSensorAdapter sensorAdapter;

    [Header("Competing controllers")]
    [Tooltip("Например RobotBrain. Эти компоненты отключаются только пока ROS bridge активен.")]
    [SerializeField] private Behaviour[] disableWhileRosIsActive = Array.Empty<Behaviour>();

    [Header("Topics")]
    [SerializeField] private string cmdVelTopic = "/gfsx/cmd_vel";
    [SerializeField] private string gripperCommandTopic = "/gfsx/gripper/command";
    [SerializeField] private string ultrasonicTopic = "/gfsx/ultrasonic/front";
    [SerializeField] private string leftIrTopic = "/gfsx/ir/left";
    [SerializeField] private string rightIrTopic = "/gfsx/ir/right";
    [SerializeField] private string gripperIrTopic = "/gfsx/ir/gripper";
    [SerializeField] private string hasBallTopic = "/gfsx/gripper/has_ball";
    [SerializeField] private string ultrasonicFrameId = "gfsx_ultrasonic_front";

    [Header("Safety and publishing")]
    [SerializeField, Min(0.05f)] private float commandTimeoutSeconds = 0.5f;
    [SerializeField, Range(1f, 60f)] private float sensorPublishRateHz = 10f;
    [SerializeField, Range(0f, 180f)] private float ultrasonicFieldOfViewDegrees = 30f;
    [SerializeField, Min(0f)] private float ultrasonicMinRangeMeters = 0.02f;

    private ROSConnection ros;
    private float lastCommandTime;
    private float nextPublishTime;
    private bool hasReceivedCommand;
    private readonly Dictionary<Behaviour, bool> previousBehaviourStates = new Dictionary<Behaviour, bool>();

    public void Configure(
        GfsxDriveAdapter drive,
        GfsxGripperAdapter gripper,
        GfsxSensorAdapter sensors,
        Behaviour[] behavioursToDisable)
    {
        driveAdapter = drive;
        gripperAdapter = gripper;
        sensorAdapter = sensors;
        disableWhileRosIsActive = behavioursToDisable ?? Array.Empty<Behaviour>();
    }

    private void Awake()
    {
        if (driveAdapter == null) driveAdapter = GetComponent<GfsxDriveAdapter>();
        if (gripperAdapter == null) gripperAdapter = GetComponent<GfsxGripperAdapter>();
        if (sensorAdapter == null) sensorAdapter = GetComponent<GfsxSensorAdapter>();

        ApplyEnvironmentOverrides();
        ros = ROSConnection.GetOrCreateInstance();
        ros.RosIPAddress = rosIpAddress;
        ros.RosPort = rosPort;

        // Подключением управляет bridge, чтобы редакторский P2/P3 setup мог безопасно
        // оставлять ConnectOnStart выключенным для ML-Agents.
        ros.ConnectOnStart = false;
        DisableCompetingControllers();
    }

    private void Start()
    {
        ros.Subscribe<TwistMsg>(cmdVelTopic, OnCmdVel);
        ros.Subscribe<BoolMsg>(gripperCommandTopic, OnGripperCommand);

        ros.RegisterPublisher<RangeMsg>(ultrasonicTopic);
        ros.RegisterPublisher<BoolMsg>(leftIrTopic);
        ros.RegisterPublisher<BoolMsg>(rightIrTopic);
        ros.RegisterPublisher<BoolMsg>(gripperIrTopic);
        ros.RegisterPublisher<BoolMsg>(hasBallTopic);

        if (connectOnStart)
        {
            ros.Connect(rosIpAddress, rosPort);
        }

        nextPublishTime = Time.unscaledTime;
        Debug.Log($"GFS-X ROS1 bridge: {rosIpAddress}:{rosPort}, cmd_vel={cmdVelTopic}", this);
    }

    private void Update()
    {
        if (hasReceivedCommand && Time.unscaledTime - lastCommandTime > commandTimeoutSeconds)
        {
            driveAdapter?.StopRobot();
            hasReceivedCommand = false;
        }

        if (Time.unscaledTime >= nextPublishTime)
        {
            PublishSensors();
            nextPublishTime = Time.unscaledTime + 1f / Mathf.Max(1f, sensorPublishRateHz);
        }
    }

    private void OnDisable()
    {
        driveAdapter?.StopRobot();
        RestoreCompetingControllers();
    }

    private void OnCmdVel(TwistMsg message)
    {
        if (message?.linear == null || message.angular == null)
        {
            return;
        }

        driveAdapter?.ApplyTwist((float)message.linear.x, (float)message.angular.z);
        lastCommandTime = Time.unscaledTime;
        hasReceivedCommand = true;
    }

    private void OnGripperCommand(BoolMsg message)
    {
        if (message != null)
        {
            gripperAdapter?.ApplyClosedCommand(message.data);
        }
    }

    private void PublishSensors()
    {
        if (sensorAdapter != null && sensorAdapter.TryGetUltrasonicMeters(out float distance))
        {
            float maxRange = sensorAdapter.UltrasonicMaxDistanceMeters;
            var rangeMessage = new RangeMsg
            {
                radiation_type = RangeMsg.ULTRASOUND,
                field_of_view = ultrasonicFieldOfViewDegrees * Mathf.Deg2Rad,
                min_range = ultrasonicMinRangeMeters,
                max_range = maxRange,
                range = Mathf.Clamp(distance, ultrasonicMinRangeMeters, maxRange)
            };
            rangeMessage.header.frame_id = ultrasonicFrameId;
            ros.Publish(ultrasonicTopic, rangeMessage);
        }

        if (sensorAdapter != null && sensorAdapter.TryGetLeftIr(out bool leftIr))
        {
            ros.Publish(leftIrTopic, new BoolMsg(leftIr));
        }

        if (sensorAdapter != null && sensorAdapter.TryGetRightIr(out bool rightIr))
        {
            ros.Publish(rightIrTopic, new BoolMsg(rightIr));
        }

        if (sensorAdapter != null && sensorAdapter.TryGetGripperIr(out bool gripperIr))
        {
            ros.Publish(gripperIrTopic, new BoolMsg(gripperIr));
        }

        if (gripperAdapter != null && gripperAdapter.TryGetHasBall(out bool hasBall))
        {
            ros.Publish(hasBallTopic, new BoolMsg(hasBall));
        }
    }

    private void ApplyEnvironmentOverrides()
    {
        string ipOverride = Environment.GetEnvironmentVariable("GFSX_ROS_IP");
        if (!string.IsNullOrWhiteSpace(ipOverride))
        {
            rosIpAddress = ipOverride.Trim();
        }

        string portOverride = Environment.GetEnvironmentVariable("GFSX_ROS_PORT");
        if (int.TryParse(portOverride, out int parsedPort) && parsedPort > 0 && parsedPort <= 65535)
        {
            rosPort = parsedPort;
        }
    }

    private void DisableCompetingControllers()
    {
        previousBehaviourStates.Clear();
        foreach (Behaviour behaviour in disableWhileRosIsActive)
        {
            if (behaviour == null || behaviour == this)
            {
                continue;
            }

            previousBehaviourStates[behaviour] = behaviour.enabled;
            behaviour.enabled = false;
        }
    }

    private void RestoreCompetingControllers()
    {
        foreach (KeyValuePair<Behaviour, bool> pair in previousBehaviourStates)
        {
            Behaviour behaviour = pair.Key;
            if (behaviour != null)
            {
                behaviour.enabled = pair.Value;
            }
        }

        previousBehaviourStates.Clear();
    }
}
