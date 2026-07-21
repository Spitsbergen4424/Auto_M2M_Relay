using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

public readonly struct DiagnosticStep
{
    public readonly int Step;
    public readonly bool BallSeen;
    public readonly float BallAngle;
    public readonly float BallDist;
    public readonly float Ultrasonic;
    public readonly float LeftIr;
    public readonly float RightIr;
    public readonly float GripperIr;
    public readonly float SensorDataAgeMs;
    public readonly float GripperIrAgeMs;
    public readonly float PwmAgeMs;
    public readonly float YoloPacketAgeMs;
    public readonly float YoloInferenceMs;
    public readonly float ActionAgeMs;
    public readonly float CameraYaw;
    public readonly float PpoGas;
    public readonly float PpoSteering;
    public readonly bool HasBall;
    public readonly int HoldTicks;
    public readonly bool IsRetrying;
    public readonly float RequestedLinearX;
    public readonly float RequestedAngularZ;
    public readonly float SentLinearX;
    public readonly float SentAngularZ;
    public readonly bool SafetyStopped;
    public readonly bool EmergencyStop;
    public readonly bool MotorCommandsEnabled;
    public readonly bool DryRun;
    public readonly float DisplacementX;
    public readonly float DisplacementZ;
    public readonly float Heading;
    public readonly float Speed;

    public DiagnosticStep(
        int step,
        bool ballSeen,
        float ballAngle,
        float ballDist,
        float ultrasonic,
        float leftIr,
        float rightIr,
        float gripperIr,
        float sensorDataAgeMs,
        float gripperIrAgeMs,
        float pwmAgeMs,
        float yoloPacketAgeMs,
        float yoloInferenceMs,
        float actionAgeMs,
        float cameraYaw,
        float ppoGas,
        float ppoSteering,
        bool hasBall,
        int holdTicks,
        bool isRetrying,
        float requestedLinearX,
        float requestedAngularZ,
        float sentLinearX,
        float sentAngularZ,
        bool safetyStopped,
        bool emergencyStop,
        bool motorCommandsEnabled,
        bool dryRun,
        float displacementX,
        float displacementZ,
        float heading,
        float speed)
    {
        Step = step;
        BallSeen = ballSeen;
        BallAngle = ballAngle;
        BallDist = ballDist;
        Ultrasonic = ultrasonic;
        LeftIr = leftIr;
        RightIr = rightIr;
        GripperIr = gripperIr;
        SensorDataAgeMs = sensorDataAgeMs;
        GripperIrAgeMs = gripperIrAgeMs;
        PwmAgeMs = pwmAgeMs;
        YoloPacketAgeMs = yoloPacketAgeMs;
        YoloInferenceMs = yoloInferenceMs;
        ActionAgeMs = actionAgeMs;
        CameraYaw = cameraYaw;
        PpoGas = ppoGas;
        PpoSteering = ppoSteering;
        HasBall = hasBall;
        HoldTicks = holdTicks;
        IsRetrying = isRetrying;
        RequestedLinearX = requestedLinearX;
        RequestedAngularZ = requestedAngularZ;
        SentLinearX = sentLinearX;
        SentAngularZ = sentAngularZ;
        SafetyStopped = safetyStopped;
        EmergencyStop = emergencyStop;
        MotorCommandsEnabled = motorCommandsEnabled;
        DryRun = dryRun;
        DisplacementX = displacementX;
        DisplacementZ = displacementZ;
        Heading = heading;
        Speed = speed;
    }
}

/// <summary>
/// P6 diagnostic telemetry logger.
///
/// Attach this component to the same GameObject as RobotBrain and enable
/// "Enable Logging" in the Inspector. It writes diagnostic_log.csv to the
/// Unity project root, next to the Assets directory.
/// </summary>
public sealed class DiagnosticLogger : MonoBehaviour
{
    [Header("P6 CSV logging")]
    [SerializeField] private bool enableLogging = false;

    [Min(1)]
    [SerializeField] private int logEveryN = 1;

    [Min(1)]
    [SerializeField] private int maxRows = 2000;

    [Min(1)]
    [SerializeField] private int flushEveryNRows = 10;

    [SerializeField] private string fileName = "diagnostic_log.csv";

    private StreamWriter writer;
    private int callsReceived;
    private int rowsWritten;
    private float startTime;
    private string outputPath;
    private bool loggingFailed;
    private bool limitReported;

    public bool EnableLogging
    {
        get => enableLogging;
        set
        {
            enableLogging = value;
            if (enableLogging && writer == null && !loggingFailed && rowsWritten < maxRows)
            {
                OpenLog();
            }
            else if (!enableLogging)
            {
                CloseLog();
            }
        }
    }

    public string OutputPath => outputPath;
    public int RowsWritten => rowsWritten;

    private void Start()
    {
        if (enableLogging)
        {
            OpenLog();
        }
    }

    /// <summary>
    /// Opens a new CSV. A previous file with the same name is overwritten.
    /// </summary>
    public void OpenLog()
    {
        if (writer != null || loggingFailed || rowsWritten >= maxRows)
        {
            return;
        }

        try
        {
            string safeFileName = string.IsNullOrWhiteSpace(fileName)
                ? "diagnostic_log.csv"
                : Path.GetFileName(fileName);

            outputPath = Path.GetFullPath(
                Path.Combine(Application.dataPath, "..", safeFileName));

            writer = new StreamWriter(
                outputPath,
                false,
                new UTF8Encoding(false));

            writer.WriteLine(
                "time,step,ballSeen,ballAngle,ballDist,uz,irL,irR,gripIR,sensorDataAgeMs,gripperIrAgeMs,pwmAgeMs,yoloPacketAgeMs,yoloInferenceMs,actionAgeMs,camYaw,ppoGas,ppoSteering,hasBall,holdTicks,isRetrying,requestedLinearX,requestedAngularZ,sentLinearX,sentAngularZ,safetyStopped,emergencyStop,motorCommandsEnabled,dryRun,displacementX,displacementZ,heading,speed");
            writer.Flush();

            callsReceived = 0;
            rowsWritten = 0;
            startTime = Time.time;
            limitReported = false;

            Debug.Log($"[DiagnosticLogger] P6 logging started: {outputPath}", this);
        }
        catch (Exception exception)
        {
            loggingFailed = true;
            Debug.LogError(
                $"[DiagnosticLogger] Could not create the CSV file: {exception.Message}",
                this);
        }
    }

    /// <summary>
    /// Writes one row of telemetry for every N-th ML-Agents decision.
    /// </summary>
    public void LogStep(DiagnosticStep step)
    {
        if (!enableLogging || loggingFailed || rowsWritten >= maxRows)
        {
            ReportLimitOnce();
            return;
        }

        if (writer == null)
        {
            OpenLog();
            if (writer == null)
            {
                return;
            }
        }

        callsReceived++;
        int effectiveLogEveryN = Mathf.Max(1, logEveryN);
        if ((callsReceived - 1) % effectiveLogEveryN != 0)
        {
            return;
        }

        float elapsed = Time.time - startTime;
        string line = string.Format(
            CultureInfo.InvariantCulture,
            "{0:F3},{1},{2},{3:F4},{4:F4},{5:F4},{6:F4},{7:F4},{8:F4},{9:F4},{10:F4},{11:F4},{12:F4},{13:F4},{14:F4},{15:F4},{16:F4},{17:F4},{18},{19},{20},{21:F4},{22:F4},{23:F4},{24:F4},{25},{26},{27},{28},{29:F4},{30:F4},{31:F4},{32:F4}",
            elapsed,
            step.Step,
            step.BallSeen ? 1 : 0,
            Sanitize(step.BallAngle),
            Sanitize(step.BallDist),
            Sanitize(step.Ultrasonic),
            Sanitize(step.LeftIr),
            Sanitize(step.RightIr),
            Sanitize(step.GripperIr),
            AgeMsOrUnavailable(step.SensorDataAgeMs),
            AgeMsOrUnavailable(step.GripperIrAgeMs),
            AgeMsOrUnavailable(step.PwmAgeMs),
            AgeMsOrUnavailable(step.YoloPacketAgeMs),
            AgeMsOrUnavailable(step.YoloInferenceMs),
            AgeMsOrUnavailable(step.ActionAgeMs),
            Sanitize(step.CameraYaw),
            Sanitize(step.PpoGas),
            Sanitize(step.PpoSteering),
            step.HasBall ? 1 : 0,
            step.HoldTicks,
            step.IsRetrying ? 1 : 0,
            Sanitize(step.RequestedLinearX),
            Sanitize(step.RequestedAngularZ),
            Sanitize(step.SentLinearX),
            Sanitize(step.SentAngularZ),
            step.SafetyStopped ? 1 : 0,
            step.EmergencyStop ? 1 : 0,
            step.MotorCommandsEnabled ? 1 : 0,
            step.DryRun ? 1 : 0,
            Sanitize(step.DisplacementX),
            Sanitize(step.DisplacementZ),
            Sanitize(step.Heading),
            Sanitize(step.Speed));

        try
        {
            writer.WriteLine(line);
            rowsWritten++;

            int effectiveFlushEveryN = Mathf.Max(1, flushEveryNRows);
            if (rowsWritten % effectiveFlushEveryN == 0 || rowsWritten >= maxRows)
            {
                writer.Flush();
            }

            if (rowsWritten >= maxRows)
            {
                ReportLimitOnce();
                CloseLog();
            }
        }
        catch (Exception exception)
        {
            loggingFailed = true;
            Debug.LogError(
                $"[DiagnosticLogger] CSV write failed: {exception.Message}",
                this);
            CloseLog();
        }
    }

    public void CloseLog()
    {
        if (writer == null)
        {
            return;
        }

        try
        {
            writer.Flush();
            writer.Dispose();
        }
        catch (Exception exception)
        {
            Debug.LogWarning(
                $"[DiagnosticLogger] CSV close warning: {exception.Message}",
                this);
        }
        finally
        {
            writer = null;
        }
    }

    private void ReportLimitOnce()
    {
        if (!limitReported && rowsWritten >= maxRows)
        {
            limitReported = true;
            Debug.Log(
                $"[DiagnosticLogger] P6 logging finished after {rowsWritten} rows. File: {outputPath}",
                this);
        }
    }

    private static float Sanitize(float value)
    {
        return float.IsNaN(value) || float.IsInfinity(value) ? 0f : value;
    }

    private static float AgeMsOrUnavailable(float value)
    {
        return value < 0f || float.IsNaN(value) || float.IsInfinity(value)
            ? -1f
            : value;
    }

    private void OnApplicationQuit()
    {
        CloseLog();
    }

    private void OnDestroy()
    {
        CloseLog();
    }
}
