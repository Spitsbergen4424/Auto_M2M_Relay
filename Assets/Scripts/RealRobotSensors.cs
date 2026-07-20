using UnityEngine;

[DisallowMultipleComponent]
public sealed class RealRobotSensors : MonoBehaviour, IRobotSensorSource
{
    [Header("Calibration")]
    [SerializeField, Min(0.01f)] private float ultrasonicMaxDistanceMeters = 1.2f;
    [SerializeField, Min(0.05f)] private float freshnessTimeoutSeconds = 0.5f;

    [Header("Diagnostics")]
    [SerializeField] private float ultrasonicMeters;
    [SerializeField] private float leftIr;
    [SerializeField] private float rightIr;
    [SerializeField] private float gripperIr;
    [SerializeField] private float leftTrackPwm;
    [SerializeField] private float rightTrackPwm;
    [SerializeField] private float lastRosPacketTime = float.NegativeInfinity;

    public float UltrasonicMaxDistanceMeters => ultrasonicMaxDistanceMeters;
    public float FreshnessTimeoutSeconds => freshnessTimeoutSeconds;
    public float UltrasonicMeters => ultrasonicMeters;
    public float UltrasonicNormalized => Mathf.Clamp01(ultrasonicMeters / Mathf.Max(0.01f, ultrasonicMaxDistanceMeters));
    public float LeftIr => leftIr;
    public float RightIr => rightIr;
    public float GripperIr => gripperIr;
    public float LeftTrackPwm => leftTrackPwm;
    public float RightTrackPwm => rightTrackPwm;
    public float LastRosPacketTime => lastRosPacketTime;
    public float LastPacketAgeSeconds => float.IsNegativeInfinity(lastRosPacketTime)
        ? float.PositiveInfinity
        : Time.unscaledTime - lastRosPacketTime;
    public bool IsDataFresh => LastPacketAgeSeconds <= freshnessTimeoutSeconds;

    public void Configure(float maxDistanceMeters)
    {
        ultrasonicMaxDistanceMeters = Mathf.Max(0.01f, maxDistanceMeters);
    }

    public void ConfigureFreshnessTimeout(float timeoutSeconds)
    {
        freshnessTimeoutSeconds = Mathf.Max(0.05f, timeoutSeconds);
    }

    public void ApplySensorData(float ultrasonicDistanceMeters, float leftIrValue, float rightIrValue,
        float gripperIrValue)
    {
        if (IsFinite(ultrasonicDistanceMeters))
        {
            ultrasonicMeters = Mathf.Max(0f, ultrasonicDistanceMeters);
        }

        leftIr = NormalizeBinary(leftIrValue);
        rightIr = NormalizeBinary(rightIrValue);
        gripperIr = NormalizeBinary(gripperIrValue);
        lastRosPacketTime = Time.unscaledTime;
    }

    public void ApplyGripperIr(int gripperIrValue)
    {
        gripperIr = gripperIrValue >= 1 ? 1f : 0f;
        lastRosPacketTime = Time.unscaledTime;
    }

    public void ApplyTrackPwm(float leftPwm, float rightPwm)
    {
        leftTrackPwm = leftPwm;
        rightTrackPwm = rightPwm;
        lastRosPacketTime = Time.unscaledTime;
    }

    public void MarkPacketReceived()
    {
        lastRosPacketTime = Time.unscaledTime;
    }

    private static float NormalizeBinary(float value)
    {
        return value >= 0.5f ? 1f : 0f;
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
