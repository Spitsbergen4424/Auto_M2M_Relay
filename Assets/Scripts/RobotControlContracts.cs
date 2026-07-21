using UnityEngine;

public interface IRobotSensorSource
{
    float UltrasonicNormalized { get; }
    float LeftIr { get; }
    float RightIr { get; }
    float GripperIr { get; }
    bool IsDataFresh { get; }
    float LastPacketAgeSeconds { get; }
}

public interface IRobotPoseSource
{
    Vector3 RelativePositionMeters { get; }
    float HeadingDegrees { get; }
    float LinearSpeedMetersPerSecond { get; }
    float AngularSpeedRadiansPerSecond { get; }
    float MaxLinearSpeedMetersPerSecond { get; }
    bool HasPoseEstimate { get; }
    void ResetPoseEstimate();
}

public interface IRobotCaptureSource
{
    bool HasCapturedBall { get; }
}

public readonly struct RobotActionCommand
{
    public readonly float Gas;
    public readonly float Steer;
    // Absolute normalized camera yaw expected by /cmd_camera_pan: -1 = left,
    // 0 = centre, 1 = right. It is deliberately not the PPO turn-rate action.
    public readonly float CameraPan;
    public readonly int GripperCommand;

    public RobotActionCommand(float gas, float steer, float cameraPan, int gripperCommand)
    {
        Gas = gas;
        Steer = steer;
        CameraPan = cameraPan;
        GripperCommand = gripperCommand;
    }
}
