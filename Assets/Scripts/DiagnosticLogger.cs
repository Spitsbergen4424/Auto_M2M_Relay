using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>Writes selected robot telemetry to a CSV file for diagnostics.</summary>
public sealed class DiagnosticLogger : MonoBehaviour
{
    [Header("CSV logging")]
    [SerializeField] private bool enableLogging;
    [SerializeField, Min(1)] private int logEveryN = 1;
    [SerializeField, Min(1)] private int maxRows = 2000;
    [SerializeField, Min(1)] private int flushEveryNRows = 1;
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
            outputPath = Path.GetFullPath(Path.Combine(Application.dataPath, "..", safeFileName));
            writer = new StreamWriter(outputPath, false, new UTF8Encoding(false));
            writer.WriteLine(
                "time,step,ballSeen,ballAngle,ballDist,uz,irL,irR,gripIR,camYaw,gas,steering,hasBall,holdTicks,isRetrying,displacementX,displacementZ,heading,speed");
            writer.Flush();
            callsReceived = 0;
            rowsWritten = 0;
            startTime = Time.time;
            limitReported = false;
            Debug.Log($"[DiagnosticLogger] Logging started: {outputPath}", this);
        }
        catch (Exception exception)
        {
            loggingFailed = true;
            Debug.LogError($"[DiagnosticLogger] Could not create CSV: {exception.Message}", this);
        }
    }

    public void LogStep(int step, bool ballSeen, float ballAngle, float ballDist,
        float ultrasonicDist, float leftIr, float rightIr, float gripperIr, float cameraYaw,
        float gas, float steering, bool hasBall, int holdTicks, bool isRetrying,
        float displacementX, float displacementZ, float heading, float speed)
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
        if ((callsReceived - 1) % Mathf.Max(1, logEveryN) != 0)
        {
            return;
        }

        string line = string.Format(CultureInfo.InvariantCulture,
            "{0:F3},{1},{2},{3:F4},{4:F4},{5:F4},{6:F4},{7:F4},{8:F4},{9:F4},{10:F4},{11:F4},{12},{13},{14},{15:F4},{16:F4},{17:F4},{18:F4}",
            Time.time - startTime, step, ballSeen ? 1 : 0, Sanitize(ballAngle),
            Sanitize(ballDist), Sanitize(ultrasonicDist), Sanitize(leftIr),
            Sanitize(rightIr), Sanitize(gripperIr), Sanitize(cameraYaw), Sanitize(gas),
            Sanitize(steering), hasBall ? 1 : 0, holdTicks, isRetrying ? 1 : 0,
            Sanitize(displacementX), Sanitize(displacementZ), Sanitize(heading), Sanitize(speed));

        try
        {
            writer.WriteLine(line);
            rowsWritten++;
            if (rowsWritten % Mathf.Max(1, flushEveryNRows) == 0 || rowsWritten >= maxRows)
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
            Debug.LogError($"[DiagnosticLogger] CSV write failed: {exception.Message}", this);
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
            Debug.LogWarning($"[DiagnosticLogger] CSV close warning: {exception.Message}", this);
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
            Debug.Log($"[DiagnosticLogger] Logging completed after {rowsWritten} rows: {outputPath}", this);
        }
    }

    private static float Sanitize(float value)
    {
        return float.IsNaN(value) || float.IsInfinity(value) ? 0f : value;
    }

    private void OnApplicationQuit() => CloseLog();
    private void OnDestroy() => CloseLog();
}
