using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

// Sits on the robot alongside TrackController/GripperController and continuously
// mirrors whatever the simulation is doing - WASD or AI driven - to the real
// robot over ROS. It reads state, it is never told what to send.
[RequireComponent(typeof(TrackController))]
public sealed class ROSBridge : MonoBehaviour
{
    [Header("Mirrored subsystems")]
    [SerializeField] private TrackController trackController;
    [SerializeField] private GripperController gripperController;
    [SerializeField] private Transform cameraPivot;

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
    [SerializeField] private float publishRate = 20f; // Hz, matches typical /cmd_vel teleop rates

    [Header("Watchdog / Fail-safe")]
    [SerializeField] private float watchdogTimeout = 0.5f;

    private ROSConnection ros;
    private float smoothGas;
    private float smoothSteering;
    private float lastPublishTime;
    private float lastLiveCommandTime;
    private bool watchdogTripped;
    private bool lastHasBall;
    private float lastCameraYaw;

    private void Awake()
    {
        trackController ??= GetComponent<TrackController>();
        gripperController ??= GetComponent<GripperController>();
        cameraPivot ??= transform.Find("CameraPivot");
    }

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
        ros.RegisterPublisher<Int32Msg>(cmdGripperTopic);
        ros.RegisterPublisher<Float32Msg>(cmdCameraPanTopic);

        lastPublishTime = Time.time;
        lastLiveCommandTime = Time.time;
        lastHasBall = gripperController != null && gripperController.HasBall;
        lastCameraYaw = CurrentCameraYaw();
    }

    private void Update()
    {
        MirrorGripper();
        MirrorCamera();

        if (Time.time - lastPublishTime >= 1f / Mathf.Max(1f, publishRate))
        {
            lastPublishTime = Time.time;
            MirrorDrive();
        }

        // If the simulated robot is genuinely idle nothing above marks fresh
        // activity, so once the deadline passes force an explicit emergency stop
        // instead of trusting whatever velocity was last on the wire.
        if (!watchdogTripped && Time.time - lastLiveCommandTime > watchdogTimeout)
        {
            watchdogTripped = true;
            SendStop();
            Debug.LogWarning($"[ROSBridge] Watchdog triggered: no live command for {watchdogTimeout:0.00}s. " +
                              "Sent emergency stop to the robot.");
        }
    }

    private void MirrorDrive()
    {
        if (trackController == null)
        {
            return;
        }

        float gas = trackController.GasCommand;
        float steer = trackController.SteerCommand;

        if (Mathf.Approximately(gas, 0f) && Mathf.Approximately(steer, 0f))
        {
            // A deliberate zero command must land immediately, not decay through the
            // filter, otherwise leftover EMA momentum drifts the robot after a stop.
            smoothGas = 0f;
            smoothSteering = 0f;
        }
        else
        {
            smoothGas = emaAlpha * gas + (1f - emaAlpha) * smoothGas;
            smoothSteering = emaAlpha * steer + (1f - emaAlpha) * smoothSteering;
            lastLiveCommandTime = Time.time;
            watchdogTripped = false;
        }

        SendTwist(smoothGas * maxLinearSpeed, smoothSteering * maxAngularSpeed);
    }

    private void MirrorGripper()
    {
        if (gripperController == null)
        {
            return;
        }

        bool hasBall = gripperController.HasBall;
        if (hasBall == lastHasBall)
        {
            return;
        }

        lastHasBall = hasBall;
        lastLiveCommandTime = Time.time;
        watchdogTripped = false;
        ros.Publish(cmdGripperTopic, new Int32Msg(hasBall ? 1 : 2));
    }

    private void MirrorCamera()
    {
        if (cameraPivot == null)
        {
            return;
        }

        float yaw = CurrentCameraYaw();
        if (Mathf.Approximately(yaw, lastCameraYaw))
        {
            return;
        }

        lastCameraYaw = yaw;
        lastLiveCommandTime = Time.time;
        watchdogTripped = false;
        ros.Publish(cmdCameraPanTopic, new Float32Msg(yaw));
    }

    private float CurrentCameraYaw()
    {
        // CameraPivot resets to identity every episode and only rotates around the
        // robot's up axis (see RobotBrain.UpdateCameraServo), so the wrapped local Y
        // euler angle is exactly the accumulated servo angle in degrees.
        return Mathf.DeltaAngle(0f, cameraPivot.localEulerAngles.y);
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
