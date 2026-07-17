using UnityEngine;

public sealed class GripperController : MonoBehaviour
{
    [SerializeField] private Transform holdPoint;
    [SerializeField] private VirtualSensors sensors;

    private GameObject heldBall;
    private Rigidbody heldBody;
    private Collider[] heldColliders;
    private bool[] colliderEnabledStates;
    private Transform originalParent;
    private bool originalKinematic;

    public bool HasBall => heldBall != null;
    public Transform HoldPoint => holdPoint;

    public void Configure(Transform point, VirtualSensors virtualSensors, float captureRadius = 0.70f)
    {
        holdPoint = point;
        sensors = virtualSensors;
        EnsureHoldPointBetweenJaws();
    }

    private void Awake()
    {
        EnsureHoldPointBetweenJaws();
    }

    private void OnValidate()
    {
        EnsureHoldPointBetweenJaws();
    }

    public void ApplyCommand(int command)
    {
        if (command == 1)
        {
            TryGrab();
        }
        else if (command == 2)
        {
            Release();
        }
    }

    public void TryGrab()
    {
        // Logical capture is allowed only after the IR ray in the claw sees the ball.
        if (HasBall || holdPoint == null || sensors == null || sensors.GripperIR < 1f)
        {
            return;
        }

        GameObject candidate = sensors.DetectedBall;
        if (candidate == null)
        {
            return;
        }

        heldBall = candidate;
        heldBody = heldBall.GetComponent<Rigidbody>();
        heldColliders = heldBall.GetComponentsInChildren<Collider>(true);
        colliderEnabledStates = new bool[heldColliders.Length];
        originalParent = heldBall.transform.parent;
        if (heldBody != null)
        {
            originalKinematic = heldBody.isKinematic;
            heldBody.linearVelocity = Vector3.zero;
            heldBody.angularVelocity = Vector3.zero;
            heldBody.isKinematic = true;
        }

        for (int i = 0; i < heldColliders.Length; i++)
        {
            colliderEnabledStates[i] = heldColliders[i].enabled;
            heldColliders[i].enabled = false;
        }

        Vector3 ballWorldScale = heldBall.transform.lossyScale;
        heldBall.transform.SetParent(holdPoint, false);
        heldBall.transform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity);
        SetWorldScale(heldBall.transform, ballWorldScale);
    }

    public GameObject Release()
    {
        if (!HasBall)
        {
            return null;
        }

        GameObject released = heldBall;
        Vector3 ballWorldScale = released.transform.lossyScale;
        released.transform.SetParent(originalParent, true);
        SetWorldScale(released.transform, ballWorldScale);
        for (int i = 0; i < heldColliders.Length; i++)
        {
            if (heldColliders[i] != null)
            {
                heldColliders[i].enabled = colliderEnabledStates[i];
            }
        }

        if (heldBody != null)
        {
            heldBody.isKinematic = originalKinematic;
            heldBody.linearVelocity = Vector3.zero;
            heldBody.angularVelocity = Vector3.zero;
            heldBody.WakeUp();
        }

        heldBall = null;
        heldBody = null;
        heldColliders = null;
        colliderEnabledStates = null;
        originalParent = null;
        return released;
    }

    private static void SetWorldScale(Transform item, Vector3 worldScale)
    {
        Vector3 parentScale = item.parent != null ? item.parent.lossyScale : Vector3.one;
        item.localScale = new Vector3(
            parentScale.x != 0f ? worldScale.x / parentScale.x : item.localScale.x,
            parentScale.y != 0f ? worldScale.y / parentScale.y : item.localScale.y,
            parentScale.z != 0f ? worldScale.z / parentScale.z : item.localScale.z);
    }

    private void EnsureHoldPointBetweenJaws()
    {
        if (holdPoint == null) return;

        Transform robotRoot = transform.root;
        Transform jawRoot = robotRoot.Find("Cube.003/Circle.002");
        if (jawRoot == null || holdPoint.parent == jawRoot) return;

        // Circle.002 is the common parent of the two jaw halves, so its origin is
        // the stable midpoint of the opening even while the gripper moves.
        holdPoint.SetParent(jawRoot, false);
        holdPoint.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity);
    }
}
