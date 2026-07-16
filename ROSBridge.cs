using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public sealed class ROSBridge : MonoBehaviour
{
    [Header("Topics")]
    [SerializeField] private string cmdVelTopic = "/cmd_vel";
    [SerializeField] private string cmdGripperTopic = "/cmd_gripper";
    [SerializeField] private string cmdCameraPanTopic = "/cmd_camera_pan";

    [Header("Robot limits")]
    [SerializeField] private float maxLinearSpeed = 0.5f;  // m/s
    [SerializeField] private float maxAngularSpeed = 1.0f; // rad/s

    [Header("Smoothing")]
    [Range(0.1f, 1f)]
    [SerializeField] private float emaAlpha = 0.8f;

    [Header("Watchdog / Fail-safe")]
    [SerializeField] private float watchdogTimeout = 0.5f;

    private ROSConnection ros;
    private float smoothGas;
    private float smoothSteering;
    private float lastCommandTime;
    private bool watchdogTripped;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
        ros.RegisterPublisher<Int32Msg>(cmdGripperTopic);
        ros.RegisterPublisher<Float32Msg>(cmdCameraPanTopic);

        lastCommandTime = Time.time;
    }

    private void Update()
    {
        // If the AI/heuristic layer stops calling PublishCommand (frozen state,
        // dropped Wi-Fi, disabled behaviour) the last velocity would otherwise keep
        // executing forever on the Raspberry Pi. Force a stop once the deadline passes.
        if (!watchdogTripped && Time.time - lastCommandTime > watchdogTimeout)
        {
            watchdogTripped = true;
            SendStop();
            Debug.LogWarning($"[ROSBridge] Watchdog triggered: no command for {watchdogTimeout:0.00}s. " +
                              "Sent emergency stop to the robot.");
        }
    }

    public void PublishCommand(float gas, float steering)
    {
        lastCommandTime = Time.time;
        watchdogTripped = false;

        if (Mathf.Approximately(gas, 0f) && Mathf.Approximately(steering, 0f))
        {
            // A deliberate zero command must land immediately, not decay through the
            // filter, otherwise leftover EMA momentum drifts the robot after a stop.
            smoothGas = 0f;
            smoothSteering = 0f;
        }
        else
        {
            smoothGas = emaAlpha * gas + (1f - emaAlpha) * smoothGas;
            smoothSteering = emaAlpha * steering + (1f - emaAlpha) * smoothSteering;
        }

        SendTwist(smoothGas * maxLinearSpeed, smoothSteering * maxAngularSpeed);
    }

    public void PublishGripperCmd(int cmd)
    {
        ros.Publish(cmdGripperTopic, new Int32Msg(cmd));
    }

    public void PublishCameraCmd(float yaw)
    {
        ros.Publish(cmdCameraPanTopic, new Float32Msg(yaw));
    }

    private void SendStop()
    {
        smoothGas = 0f;
        smoothSteering = 0f;
        SendTwist(0f, 0f);
    }

    private void SendTwist(float linearX, float angularZ)
    {
        TwistMsg cmd = new TwistMsg();
        cmd.linear.x = linearX;
        cmd.angular.z = angularZ;
        ros.Publish(cmdVelTopic, cmd);
    }
}
