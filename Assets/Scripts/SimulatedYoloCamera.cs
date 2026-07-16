using UnityEngine;

public sealed class SimulatedYoloCamera : MonoBehaviour
{
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
        EvaluateTarget();
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

        // If the first collider on the ray is not the ball, a wall or another
        // object blocks the camera's line of sight.
        if (!Physics.Raycast(origin, toBall.normalized, out RaycastHit hit, distance + 0.05f,
                visibilityMask, QueryTriggerInteraction.Ignore))
        {
            return true;
        }

        return hit.transform == targetBall || hit.transform.IsChildOf(targetBall);
    }
}
