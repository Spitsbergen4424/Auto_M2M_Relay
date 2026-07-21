using System;
using System.Collections.Concurrent;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

/// <summary>
/// Receives the latest YOLO observation from yolo_vision_node.py over UDP.
/// Distance semantics match SimulatedYoloCamera: 0 = near, 1 = far/not seen.
/// </summary>
public sealed class RealYoloCamera : YoloVisionSource
{
    [Serializable]
    private sealed class YoloPacket
    {
        public int protocol;
        public long sequence;
        public float sees;
        public float angle;
        public float distance;
        public float confidence;
        public float bboxWidth;
        public float bboxHeight;
        public float bboxHeightRatio;
        public float inferenceMs;
    }

    [Header("UDP")]
    [SerializeField, Range(1, 65535)] private int listenPort = 5005;
    [SerializeField, Min(0.1f)] private float packetTimeoutSeconds = 0.35f;

    [Header("Diagnostics")]
    [SerializeField] private bool showDebugOverlay = true;

    public override bool IsVisible { get; protected set; }
    public override float HorizontalOffset { get; protected set; }
    public override float NormalizedDistance { get; protected set; } = 1f;
    public override float LastKnownDirection { get; protected set; }
    public override float TimeSinceDetection { get; protected set; }

    public float Confidence { get; private set; }
    public float InferenceMilliseconds { get; private set; }
    public float BoundingBoxHeightRatio { get; private set; }
    public long LastSequence { get; private set; } = -1;
    public int ListenPort => listenPort;
    public float PacketTimeoutSeconds => packetTimeoutSeconds;
    public bool IsReceivingPackets => Time.unscaledTime - lastPacketTime <= packetTimeoutSeconds;
    public float LastPacketAgeSeconds => float.IsNegativeInfinity(lastPacketTime)
        ? float.PositiveInfinity
        : Time.unscaledTime - lastPacketTime;
    public float LastPacketAgeMilliseconds => AgeToMilliseconds(LastPacketAgeSeconds);

    public void Configure(int port, float timeoutSeconds)
    {
        listenPort = port;
        packetTimeoutSeconds = Mathf.Max(0.1f, timeoutSeconds);
    }

    private readonly ConcurrentQueue<YoloPacket> packetQueue = new ConcurrentQueue<YoloPacket>();
    private UdpClient udpClient;
    private Thread listenerThread;
    private volatile bool listenerRunning;
    private float lastPacketTime = float.NegativeInfinity;
    private GUIStyle debugStyle;

    private void OnEnable()
    {
        StartListener();
    }

    private void Update()
    {
        // Drain the queue and apply only the newest packet. Old vision data must
        // never delay control of a moving robot.
        YoloPacket newest = null;
        while (packetQueue.TryDequeue(out YoloPacket packet))
        {
            newest = packet;
        }

        if (newest != null)
        {
            ApplyPacket(newest);
        }

        if (!IsReceivingPackets)
        {
            MarkNotVisible();
        }
        else if (!IsVisible)
        {
            TimeSinceDetection += Time.unscaledDeltaTime;
        }
    }

    private void OnDisable()
    {
        StopListener();
        MarkNotVisible();
    }

    private void OnApplicationQuit()
    {
        StopListener();
    }

    private void StartListener()
    {
        if (listenerRunning)
        {
            return;
        }

        try
        {
            udpClient = new UdpClient(listenPort);
            udpClient.Client.ReceiveTimeout = 250;
            listenerRunning = true;
            listenerThread = new Thread(ListenLoop)
            {
                IsBackground = true,
                Name = "GFSX YOLO UDP receiver"
            };
            listenerThread.Start();
            Debug.Log($"RealYoloCamera is listening on UDP {listenPort}", this);
        }
        catch (Exception exception)
        {
            Debug.LogError($"Cannot start YOLO UDP receiver on port {listenPort}: {exception.Message}", this);
            listenerRunning = false;
        }
    }

    private void StopListener()
    {
        listenerRunning = false;
        try
        {
            udpClient?.Close();
        }
        catch (Exception)
        {
            // Socket may already be closed while Unity is stopping.
        }

        if (listenerThread != null && listenerThread.IsAlive)
        {
            listenerThread.Join(500);
        }

        listenerThread = null;
        udpClient = null;
    }

    private void ListenLoop()
    {
        while (listenerRunning)
        {
            try
            {
                byte[] bytes = udpClient.Receive(ref _unusedEndpoint);
                string json = Encoding.UTF8.GetString(bytes);
                YoloPacket packet = JsonUtility.FromJson<YoloPacket>(json);
                if (packet != null && packet.protocol == 1)
                {
                    packetQueue.Enqueue(packet);
                }
            }
            catch (SocketException exception)
            {
                // 10060/TimedOut is expected: it lets the thread notice shutdown.
                if (listenerRunning && exception.SocketErrorCode != SocketError.TimedOut)
                {
                    Debug.LogWarning($"YOLO UDP receive error: {exception.Message}", this);
                }
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            catch (Exception exception)
            {
                if (listenerRunning)
                {
                    Debug.LogWarning($"Invalid YOLO packet: {exception.Message}", this);
                }
            }
        }
    }

    private System.Net.IPEndPoint _unusedEndpoint = new System.Net.IPEndPoint(System.Net.IPAddress.Any, 0);

    private void ApplyPacket(YoloPacket packet)
    {
        if (!IsFinite(packet.angle) || !IsFinite(packet.distance))
        {
            return;
        }

        // Reject an out-of-order packet while the sender is alive, but accept a
        // reset sequence after yolo_vision_node.py has been restarted.
        if (packet.sequence < LastSequence && IsReceivingPackets)
        {
            return;
        }

        LastSequence = packet.sequence;
        lastPacketTime = Time.unscaledTime;
        Confidence = Mathf.Clamp01(packet.confidence);
        InferenceMilliseconds = Mathf.Max(0f, packet.inferenceMs);
        BoundingBoxHeightRatio = Mathf.Clamp01(packet.bboxHeightRatio);
        IsVisible = packet.sees > 0.5f;

        if (IsVisible)
        {
            HorizontalOffset = Mathf.Clamp(packet.angle, -1f, 1f);
            NormalizedDistance = Mathf.Clamp01(packet.distance);
            LastKnownDirection = HorizontalOffset;
            TimeSinceDetection = 0f;
        }
        else
        {
            HorizontalOffset = 0f;
            NormalizedDistance = 1f;
        }
    }

    private void MarkNotVisible()
    {
        IsVisible = false;
        HorizontalOffset = 0f;
        NormalizedDistance = 1f;
        Confidence = 0f;
        TimeSinceDetection += Time.unscaledDeltaTime;
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private static float AgeToMilliseconds(float ageSeconds)
    {
        return float.IsNegativeInfinity(ageSeconds) || float.IsNaN(ageSeconds) || float.IsInfinity(ageSeconds)
            ? -1f
            : ageSeconds * 1000f;
    }

    private void OnGUI()
    {
        if (!showDebugOverlay || !Application.isPlaying)
        {
            return;
        }

        debugStyle ??= new GUIStyle(GUI.skin.box)
        {
            alignment = TextAnchor.UpperLeft,
            fontSize = 16,
            normal = { textColor = Color.white }
        };

        string text = $"REAL YOLO\n" +
                      $"UDP: {(IsReceivingPackets ? "OK" : "TIMEOUT")}\n" +
                      $"Visible: {(IsVisible ? "YES" : "NO")}\n" +
                      $"Offset: {HorizontalOffset:F2}\n" +
                      $"Distance: {NormalizedDistance:F2}\n" +
                      $"Confidence: {Confidence:F2}\n" +
                      $"Box H ratio: {BoundingBoxHeightRatio:F3}\n" +
                      $"Inference: {InferenceMilliseconds:F1} ms";
        GUI.Box(new Rect(12f, 125f, 220f, 185f), text, debugStyle);
    }
}
