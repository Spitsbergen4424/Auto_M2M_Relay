using UnityEngine;

[RequireComponent(typeof(Camera))]
public sealed class RobotCameraViewport : MonoBehaviour
{
    [SerializeField, Range(0.1f, 0.4f)] private float widthFraction = 0.18f;
    [SerializeField, Range(0f, 0.1f)] private float margin = 0.02f;
    [SerializeField] private string mountPartName = "Plane.016";

    private Camera targetCamera;
    private int previousWidth;
    private int previousHeight;

    public void Configure(float width, float edgeMargin)
    {
        widthFraction = width;
        margin = edgeMargin;
        ApplyViewport();
    }

    private void Awake()
    {
        targetCamera = GetComponent<Camera>();
        MountToRobotPart();
        ConfigureOverviewCamera();
        ApplyViewport();
    }

    private void MountToRobotPart()
    {
        Transform pivot = transform.parent;
        if (pivot == null) return;

        Transform robotRoot = pivot.root;
        Transform mount = null;
        foreach (Transform item in robotRoot.GetComponentsInChildren<Transform>(true))
        {
            if (item != robotRoot && item.name.Equals(mountPartName, System.StringComparison.OrdinalIgnoreCase))
            {
                mount = item;
                break;
            }
        }

        if (mount == null || pivot.parent == mount) return;

        Vector3 cameraPosition = transform.position;
        Quaternion cameraRotation = transform.rotation;
        pivot.SetParent(mount, true);
        pivot.SetPositionAndRotation(mount.position, mount.rotation);
        transform.SetPositionAndRotation(cameraPosition, cameraRotation);
    }

    private void ConfigureOverviewCamera()
    {
        Camera overview = Camera.main;
        if (overview == null || overview == targetCamera) return;

        Transform robot = transform.root;
        overview.transform.SetParent(robot, false);
        overview.transform.localPosition = new Vector3(0f, 4.8f, -6.5f);
        overview.transform.LookAt(robot.TransformPoint(new Vector3(0f, 0.8f, 1f)));
    }

    private void LateUpdate()
    {
        if (previousWidth != Screen.width || previousHeight != Screen.height)
        {
            ApplyViewport();
        }
    }

    private void ApplyViewport()
    {
        targetCamera ??= GetComponent<Camera>();
        if (targetCamera == null || Screen.width <= 0 || Screen.height <= 0) return;

        // Camera.rect uses normalized coordinates. Compensating for the screen aspect
        // keeps the robot view square in pixels at any Game-view resolution.
        float heightFraction = Mathf.Min(0.9f, widthFraction * Screen.width / Screen.height);
        targetCamera.rect = new Rect(1f - margin - widthFraction, 1f - margin - heightFraction,
            widthFraction, heightFraction);
        previousWidth = Screen.width;
        previousHeight = Screen.height;
    }
}
