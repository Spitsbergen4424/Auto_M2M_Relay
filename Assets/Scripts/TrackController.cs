using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public sealed class TrackController : MonoBehaviour
{
    [Header("Drive calibration")]
    [SerializeField] private float moveSpeed = 0.57f;
    [SerializeField] private float turnSpeed = 120f;
    [SerializeField] private float turnK = 0.30f;
    [SerializeField] private float maxLinearCmd = 0.25f;
    [SerializeField] private float motorDeadzone = 10f;
    [SerializeField] private float minMotorPwm = 35f;
    [SerializeField] private float maxPwmStep = 15f;
    [SerializeField] private float velocityToPwm = 200f;
    [SerializeField] private float acceleration = 1.2f;
    [SerializeField] private float coastDeceleration = 0.56f;
    [SerializeField] private float brakingDeceleration = 2.4f;
    [SerializeField] private float steeringResponse = 2.0f;
    [SerializeField] private float steeringReturn = 2.8f;

    private Rigidbody body;
    private float gasCommand;
    private float steerCommand;
    private float leftPwm;
    private float rightPwm;
    private float drivePwm;
    private float currentSpeed;
    private float currentSteer;

    public float GasCommand => gasCommand;
    public float SteerCommand => steerCommand;
    public float LeftPwm => leftPwm;
    public float RightPwm => rightPwm;
    public float CurrentSpeed => currentSpeed;
    public float CurrentSteer => currentSteer;
    public float MaxLinearSpeed => maxLinearCmd;

    public void ConfigureScale(float multiplier)
    {
        moveSpeed = 0.57f * multiplier;
        maxLinearCmd = 0.25f * multiplier;
        velocityToPwm = 200f / multiplier;
        motorDeadzone = 30f;
        minMotorPwm = 50f;
        maxPwmStep = 15f;
        acceleration = 0.17f * multiplier;
        coastDeceleration = 0.08f * multiplier;
        brakingDeceleration = 0.34f * multiplier;
        steeringResponse = 2.0f;
        steeringReturn = 2.8f;
    }


    private void Awake()
    {
        body = GetComponent<Rigidbody>();
        body.mass = 2.5f;
        body.linearDamping = 8f;
        body.angularDamping = 8f;
        body.interpolation = RigidbodyInterpolation.Interpolate;
        body.collisionDetectionMode = CollisionDetectionMode.Continuous;
        body.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

    }

    public void SetCommand(float gas, float steer)
    {
        gasCommand = Mathf.Clamp(gas, -1f, 1f);
        steerCommand = Mathf.Clamp(steer, -1f, 1f);
    }

    public void Stop()
    {
        gasCommand = 0f;
        steerCommand = 0f;
        leftPwm = 0f;
        rightPwm = 0f;
        drivePwm = 0f;
        currentSpeed = 0f;
        currentSteer = 0f;
    }

    private void FixedUpdate()
    {
        float targetSpeed = Mathf.Clamp(gasCommand * moveSpeed, -maxLinearCmd, maxLinearCmd);
        bool coasting = Mathf.Abs(gasCommand) < 0.01f;
        bool reversing = !coasting && Mathf.Abs(currentSpeed) > 0.01f &&
                         Mathf.Sign(targetSpeed) != Mathf.Sign(currentSpeed);
        float speedRate = coasting ? coastDeceleration : reversing ? brakingDeceleration : acceleration;
        currentSpeed = Mathf.MoveTowards(currentSpeed, targetSpeed, speedRate * Time.fixedDeltaTime);

        float steerRate = Mathf.Abs(steerCommand) < 0.01f ? steeringReturn : steeringResponse;
        currentSteer = Mathf.MoveTowards(currentSteer, steerCommand, steerRate * Time.fixedDeltaTime);

        float leftVelocity = targetSpeed - currentSteer * turnK;
        float rightVelocity = targetSpeed + currentSteer * turnK;

        float targetLeftPwm = VelocityToMotorPwm(leftVelocity);
        float targetRightPwm = VelocityToMotorPwm(rightVelocity);
        leftPwm = Mathf.MoveTowards(leftPwm, targetLeftPwm, maxPwmStep);
        rightPwm = Mathf.MoveTowards(rightPwm, targetRightPwm, maxPwmStep);

        // Keep propulsion independent from steering so W/S always trace a straight line.
        drivePwm = Mathf.MoveTowards(drivePwm, VelocityToMotorPwm(targetSpeed), maxPwmStep);

        // This GFS-X mesh is authored nose-first along its local +X axis.
        Vector3 displacement = transform.right * (currentSpeed * Time.fixedDeltaTime);
        Quaternion rotation = Quaternion.AngleAxis(
            currentSteer * turnSpeed * Time.fixedDeltaTime, Vector3.up);
        body.MovePosition(body.position + displacement);
        body.MoveRotation(rotation * body.rotation);
    }

    private float VelocityToMotorPwm(float velocity)
    {
        float raw = Mathf.Clamp(velocity * velocityToPwm, -100f, 100f);
        float magnitude = Mathf.Abs(raw);
        if (magnitude < motorDeadzone)
        {
            return 0f;
        }

        return Mathf.Sign(raw) * Mathf.Max(magnitude, minMotorPwm);
    }
}
