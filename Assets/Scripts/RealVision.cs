using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

[Serializable]
public sealed class YoloDataPacket
{
    public float angle;
    public float distance;
    public float sees;
    public float conf;
    public float w;
    public float h;
}

[DisallowMultipleComponent]
public sealed class RealVision : MonoBehaviour
{
    [SerializeField] private SimulatedYoloCamera cameraTarget;
    [SerializeField, Min(1)] private int udpPort = 5005;
    [SerializeField, Min(0.1f)] private float staleAfterSeconds = 0.5f;
    [SerializeField] private bool externalMode;

    public bool SeesBall { get; private set; }
    public float NormalizedAngle { get; private set; }
    public float NormalizedDistance { get; private set; } = 1f;
    public float Confidence { get; private set; }

    private readonly ConcurrentQueue<YoloDataPacket> packets = new ConcurrentQueue<YoloDataPacket>();
    private CancellationTokenSource cancellation;
    private float lastPacketTime = float.NegativeInfinity;
    private bool listenerRunning;
    private string listenerError;

    public void Configure(SimulatedYoloCamera camera)
    {
        cameraTarget = camera;
    }

    public void SetExternalMode(bool enabled)
    {
        externalMode = enabled;
        if (!enabled)
        {
            ResetDetection();
        }
    }

    private void Awake()
    {
        cameraTarget ??= GetComponentInChildren<SimulatedYoloCamera>(true);
    }

    private void OnEnable()
    {
        StartListener();
    }

    private void Update()
    {
        if (!string.IsNullOrEmpty(listenerError))
        {
            Debug.LogWarning(listenerError, this);
            listenerError = null;
        }

        bool gotPacket = false;
        YoloDataPacket newest = null;
        while (packets.TryDequeue(out YoloDataPacket packet))
        {
            newest = packet;
            gotPacket = true;
        }

        if (gotPacket)
        {
            lastPacketTime = Time.realtimeSinceStartup;
            ApplyPacket(newest);
        }
        else if (externalMode && Time.realtimeSinceStartup - lastPacketTime > staleAfterSeconds)
        {
            ResetDetection();
        }
    }

    private void ApplyPacket(YoloDataPacket packet)
    {
        SeesBall = packet != null && packet.sees > 0.5f;
        Confidence = packet != null ? Mathf.Clamp01(packet.conf) : 0f;
        NormalizedAngle = SeesBall ? Mathf.Clamp(packet.angle, -1f, 1f) : 0f;

        // P7 sends box height / image height: larger means closer. The agent
        // expects 0 = close and 1 = far, so invert the P7 value here.
        float closeness = packet != null ? Mathf.Clamp01(packet.distance) : 0f;
        NormalizedDistance = SeesBall ? 1f - closeness : 1f;

        if (externalMode)
        {
            cameraTarget?.SetExternalDetection(SeesBall, NormalizedAngle, NormalizedDistance);
        }
    }

    private void ResetDetection()
    {
        SeesBall = false;
        NormalizedAngle = 0f;
        NormalizedDistance = 1f;
        Confidence = 0f;
        if (externalMode)
        {
            cameraTarget?.SetExternalDetection(false, 0f, 1f);
        }
    }

    private void StartListener()
    {
        if (listenerRunning)
        {
            return;
        }

        listenerRunning = true;
        cancellation = new CancellationTokenSource();
        _ = Task.Run(() => ListenLoop(cancellation.Token));
    }

    private void ListenLoop(CancellationToken token)
    {
        try
        {
            using var client = new UdpClient(udpPort);
            client.Client.ReceiveTimeout = 250;
            var source = new IPEndPoint(IPAddress.Any, 0);
            while (!token.IsCancellationRequested)
            {
                try
                {
                    byte[] bytes = client.Receive(ref source);
                    string json = Encoding.UTF8.GetString(bytes);
                    YoloDataPacket packet = JsonUtility.FromJson<YoloDataPacket>(json);
                    if (packet != null)
                    {
                        packets.Enqueue(packet);
                    }
                }
                catch (SocketException exception) when (exception.SocketErrorCode == SocketError.TimedOut)
                {
                    // Wake periodically to observe cancellation.
                }
                catch (Exception exception)
                {
                    listenerError = $"RealVision ignored UDP packet: {exception.Message}";
                }
            }
        }
        catch (SocketException exception)
        {
            listenerError = $"RealVision cannot listen on UDP {udpPort}: {exception.Message}";
        }
        finally
        {
            listenerRunning = false;
        }
    }

    private void OnDisable()
    {
        cancellation?.Cancel();
        cancellation?.Dispose();
        cancellation = null;
        listenerRunning = false;
    }
}
