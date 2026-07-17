using UnityEngine;

public sealed class SimulatedYoloCamera : MonoBehaviour
{
    [SerializeField] private bool useExternalDetection;
    [SerializeField] private Camera sensorCamera;
    [SerializeField] private Transform targetBall;
    [SerializeField] private float horizontalFov = 40f;
    [SerializeField] private float maxVisibleDistance = 2f;
    [SerializeField] private LayerMask visibilityMask = ~0;
    [SerializeField] private bool showDebugOverlay = true;

    public bool IsVisible { get; private set; }
    public float HorizontalOffset { get; private set; }
    public float NormalizedDistance { get; private set; } = 1f;
    public float LastKnownDirection { get; private set; }
    public float TimeSinceDetection { get; private set; }
    public bool UseExternalDetection => useExternalDetection;
    public int WorldViewSector
    {
        get
        {
            if (sensorCamera == null)
            {
                return 0;
            }

            Vector3 forward = Vector3.ProjectOnPlane(sensorCamera.transform.forward, Vector3.up);
            float yaw = Mathf.Atan2(forward.x, forward.z) * Mathf.Rad2Deg;
            float normalized = Mathf.Repeat(yaw + 180f, 360f) / 360f;
            return Mathf.Clamp(Mathf.FloorToInt(normalized * 16f), 0, 15);
        }
    }

    private GUIStyle debugStyle;

    public void Configure(Camera cameraComponent, Transform ball)
    {
        sensorCamera = cameraComponent;
        targetBall = ball;
    }

    public void ConfigureScale(float multiplier)
    {
        maxVisibleDistance = 2f * multiplier;
    }

    private void Update()
    {
        if (useExternalDetection)
        {
            if (!IsVisible)
            {
                TimeSinceDetection += Time.deltaTime;
            }
            return;
        }

        EvaluateTarget();
    }

    public void SetExternalMode(bool enabled)
    {
        useExternalDetection = enabled;
        if (!enabled) return;
        IsVisible = false;
        HorizontalOffset = 0f;
        NormalizedDistance = 1f;
    }

    public void SetExternalDetection(bool visible, float horizontalOffset, float normalizedDistance)
    {
        if (!useExternalDetection) return;
        IsVisible = visible;
        HorizontalOffset = visible ? Mathf.Clamp(horizontalOffset, -1f, 1f) : 0f;
        NormalizedDistance = visible ? Mathf.Clamp01(normalizedDistance) : 1f;
        if (visible)
        {
            LastKnownDirection = HorizontalOffset;
            TimeSinceDetection = 0f;
        }
    }

    public void MarkExternalDataStale()
    {
        if (!useExternalDetection) return;
        IsVisible = false;
        HorizontalOffset = 0f;
        NormalizedDistance = 1f;
    }

    private void OnGUI()
    {
        if (!showDebugOverlay || !Application.isPlaying) return;

        debugStyle ??= new GUIStyle(GUI.skin.box)
        {
            alignment = TextAnchor.UpperLeft,
            fontSize = 16,
            normal = { textColor = Color.white }
        };

        string visibility = IsVisible ? "YES" : "NO";
        string text = $"YOLO CAMERA\n" +
                      $"Visible: {visibility}\n" +
                      $"Offset X: {HorizontalOffset:F2}\n" +
                      $"Distance: {NormalizedDistance:F2}";
        GUI.Box(new Rect(12f, 12f, 175f, 105f), text, debugStyle);
    }

    public void EvaluateTarget()
    {
        IsVisible = false;
        HorizontalOffset = 0f;
        NormalizedDistance = 1f;

        if (sensorCamera == null || targetBall == null)
        {
            TimeSinceDetection += Time.deltaTime;
            return;
        }

        Vector3 cameraPosition = sensorCamera.transform.position;
        Vector3 toBall = targetBall.position - cameraPosition;
        float distance = toBall.magnitude;
        // Viewport coordinates are 0..1. Convert X to a signed value where
        // -1 is the left edge, 0 is the image centre and +1 is the right edge.
        Vector3 viewport = sensorCamera.WorldToViewportPoint(targetBall.position);
        float relativeHorizontalAngle = Mathf.Clamp((viewport.x - 0.5f) * 2f, -1f, 1f);
        float normalizedDistance = Mathf.Clamp01(distance / maxVisibleDistance);
        float horizontalAngle = Vector3.SignedAngle(sensorCamera.transform.forward,
            Vector3.ProjectOnPlane(toBall, sensorCamera.transform.up), sensorCamera.transform.up);

        bool inViewport = viewport.z > 0f && viewport.x >= 0f && viewport.x <= 1f &&
                          viewport.y >= 0f && viewport.y <= 1f;
        bool inHorizontalFov = Mathf.Abs(horizontalAngle) <= horizontalFov * 0.5f;
        bool clearLine = HasClearLine(cameraPosition, toBall, distance);

        if (distance <= maxVisibleDistance && inViewport && inHorizontalFov && clearLine)
        {
            IsVisible = true;
            HorizontalOffset = relativeHorizontalAngle;
            NormalizedDistance = normalizedDistance;
            LastKnownDirection = HorizontalOffset;
            TimeSinceDetection = 0f;
        }
        else
        {
            TimeSinceDetection += Time.deltaTime;
        }
    }

    private bool HasClearLine(Vector3 origin, Vector3 toBall, float distance)
    {
        if (distance <= 0.0001f)
        {
            return true;
        }

        RobotBrain owner = GetComponentInParent<RobotBrain>();
        Transform robotRoot = owner != null ? owner.transform : null;
        RaycastHit[] hits = Physics.RaycastAll(origin, toBall.normalized, distance + 0.05f,
            visibilityMask, QueryTriggerInteraction.Ignore);
        if (hits.Length == 0)
        {
            return true;
        }

        System.Array.Sort(hits, (first, second) => first.distance.CompareTo(second.distance));
        foreach (RaycastHit hit in hits)
        {
            // Body and gripper colliders belong to the observing robot, not to the scene.
            if (robotRoot != null && (hit.transform == robotRoot || hit.transform.IsChildOf(robotRoot)))
            {
                continue;
            }

            return hit.transform == targetBall || hit.transform.IsChildOf(targetBall);
        }

        return true;
    }
}
