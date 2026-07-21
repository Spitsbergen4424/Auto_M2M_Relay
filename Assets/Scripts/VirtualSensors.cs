using UnityEngine;

public sealed class VirtualSensors : MonoBehaviour, IRobotSensorSource
{
    [Header("Sensor origins")]
    [SerializeField] private Transform centerPoint;
    [SerializeField] private Transform leftIRPoint;
    [SerializeField] private Transform rightIRPoint;
    [SerializeField] private Transform gripperIRPoint;

    [Header("Ranges")]
    [SerializeField] private float ultrasonicRange = 2f;
    [SerializeField] private float ultrasonicConeAngle = 30f;
    [SerializeField, Range(3, 15)] private int ultrasonicRayCount = 7;
    [SerializeField] private float obstacleIRRange = 0.15f;
    [SerializeField] private float gripperIRRange = 0.08f;
    [SerializeField] private LayerMask detectionMask = ~0;

    [Header("Scene ray visualization")]
    [SerializeField] private bool showRaycasts = true;
    [SerializeField] private Color centerRayColor = Color.cyan;
    [SerializeField] private Color irRayColor = Color.yellow;
    [SerializeField] private Color gripperRayColor = Color.magenta;
    [SerializeField] private Color hitRayColor = Color.red;

    public float UltrasonicNormalized { get; private set; } = 1f;
    public float LeftIR { get; private set; }
    public float RightIR { get; private set; }
    public float GripperIR { get; private set; }
    public float LeftIr => LeftIR;
    public float RightIr => RightIR;
    public float GripperIr => GripperIR;
    public GameObject DetectedBall { get; private set; }
    public float LastPacketTime { get; private set; }
    public float LastPacketAgeSeconds => Time.time - LastPacketTime;
    public bool IsDataFresh => true;

    private void Awake()
    {
        LastPacketTime = Time.time;
    }

    public void ConfigureScale(float multiplier)
    {
        ultrasonicRange = 2f * multiplier;
        obstacleIRRange = 0.15f * multiplier;
        gripperIRRange = 0.08f * multiplier;
    }

    public void Configure(Transform center, Transform left, Transform right, Transform gripper)
    {
        centerPoint = center;
        leftIRPoint = left;
        rightIRPoint = right;
        gripperIRPoint = gripper;
    }

    private void FixedUpdate()
    {
        UltrasonicNormalized = ReadUltrasonic();
        LeftIR = ReadObstacleIR(leftIRPoint);
        RightIR = ReadObstacleIR(rightIRPoint);
        GripperIR = ReadGripperIR();
        LastPacketTime = Time.time;
    }

    private float ReadUltrasonic()
    {
        if (centerPoint == null)
        {
            return 1f;
        }

        float nearest = ultrasonicRange;
        for (int i = 0; i < ultrasonicRayCount; i++)
        {
            float t = ultrasonicRayCount == 1 ? 0.5f : i / (float)(ultrasonicRayCount - 1);
            float angle = Mathf.Lerp(-ultrasonicConeAngle * 0.5f, ultrasonicConeAngle * 0.5f, t);
            Vector3 direction = Quaternion.AngleAxis(angle, centerPoint.up) * centerPoint.forward;
            float distance = ClosestHit(centerPoint.position, direction, ultrasonicRange, true, out GameObject hit);
            DrawPhysicsRay(centerPoint.position, direction, ultrasonicRange, distance, hit, centerRayColor);
            nearest = Mathf.Min(nearest, distance);
        }

        return Mathf.Clamp01(nearest / ultrasonicRange);
    }

    private float ReadObstacleIR(Transform origin)
    {
        if (origin == null)
        {
            return 0f;
        }

        float distance = ClosestHit(origin.position, origin.forward, obstacleIRRange, true, out GameObject hit);
        DrawPhysicsRay(origin.position, origin.forward, obstacleIRRange, distance, hit, irRayColor);
        return distance < obstacleIRRange ? 1f : 0f;
    }

    private float ReadGripperIR()
    {
        DetectedBall = null;
        if (gripperIRPoint == null)
        {
            return 0f;
        }

        float distance = ClosestHit(gripperIRPoint.position, gripperIRPoint.forward, gripperIRRange, false,
            out GameObject hit);
        DrawPhysicsRay(gripperIRPoint.position, gripperIRPoint.forward, gripperIRRange, distance, hit,
            gripperRayColor);
        if (hit != null && hit.CompareTag("TargetBall"))
        {
            DetectedBall = hit;
            return 1f;
        }

        return 0f;
    }

    private float ClosestHit(Vector3 origin, Vector3 direction, float maxDistance, bool ignoreBall, out GameObject hitObject)
    {
        float nearest = maxDistance;
        hitObject = null;
        RaycastHit[] hits = Physics.RaycastAll(origin, direction, maxDistance, detectionMask, QueryTriggerInteraction.Ignore);
        foreach (RaycastHit hit in hits)
        {
            if (hit.transform.IsChildOf(transform) || hit.transform == transform)
            {
                continue;
            }

            if (ignoreBall && hit.collider.CompareTag("TargetBall"))
            {
                continue;
            }

            if (hit.distance < nearest)
            {
                nearest = hit.distance;
                hitObject = hit.collider.gameObject;
            }
        }

        return nearest;
    }

    private void OnDrawGizmos()
    {
        if (!showRaycasts) return;

        DrawUltrasonicGizmos();
        DrawGizmoRay(leftIRPoint, obstacleIRRange, irRayColor);
        DrawGizmoRay(rightIRPoint, obstacleIRRange, irRayColor);
        DrawGizmoRay(gripperIRPoint, gripperIRRange, gripperRayColor);
    }

    private void DrawUltrasonicGizmos()
    {
        if (centerPoint == null) return;
        for (int i = 0; i < ultrasonicRayCount; i++)
        {
            float t = ultrasonicRayCount == 1 ? 0.5f : i / (float)(ultrasonicRayCount - 1);
            float angle = Mathf.Lerp(-ultrasonicConeAngle * 0.5f, ultrasonicConeAngle * 0.5f, t);
            Vector3 direction = Quaternion.AngleAxis(angle, centerPoint.up) * centerPoint.forward;
            DrawGizmoRay(centerPoint.position, direction, ultrasonicRange, centerRayColor);
        }
    }

    private static void DrawGizmoRay(Transform origin, float distance, Color color)
    {
        if (origin == null) return;
        DrawGizmoRay(origin.position, origin.forward, distance, color);
    }

    private static void DrawGizmoRay(Vector3 origin, Vector3 direction, float distance, Color color)
    {
        Gizmos.color = color;
        Gizmos.DrawLine(origin, origin + direction * distance);
        Gizmos.DrawSphere(origin, 0.025f);
    }

    private void DrawPhysicsRay(Vector3 origin, Vector3 direction, float maxDistance, float hitDistance,
        GameObject hit, Color clearColor)
    {
        if (!showRaycasts) return;

        float visibleDistance = hit != null ? hitDistance : maxDistance;
        Debug.DrawLine(origin, origin + direction * visibleDistance, hit != null ? hitRayColor : clearColor,
            Time.fixedDeltaTime, false);
        if (hit != null)
        {
            Debug.DrawRay(origin + direction * hitDistance, direction * 0.03f, hitRayColor,
                Time.fixedDeltaTime, false);
        }
    }
}
