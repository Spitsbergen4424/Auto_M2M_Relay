using System;
using System.Reflection;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class GfsxDriveAdapter : MonoBehaviour
{
    [Header("Controller compatibility")]
    [SerializeField] private Component targetController;
    [SerializeField] private string commandMethodName = "SetCommand";

    [Header("Twist to normalized command")]
    [SerializeField, Min(0.001f)] private float maxLinearSpeed = 0.25f;
    [SerializeField, Min(0.001f)] private float maxAngularSpeed = 2.0943952f;
    [SerializeField] private bool invertLinear;
    [SerializeField] private bool invertAngular;

    private MethodInfo commandMethod;
    private bool reportedMissingMethod;

    public Component TargetController => targetController;
    public bool IsReady => targetController != null && commandMethod != null;

    public void Configure(Component controller)
    {
        targetController = controller;
        CacheMethod();
    }

    public void ApplyTwist(float linearMetersPerSecond, float angularRadiansPerSecond)
    {
        float gas = Mathf.Clamp(linearMetersPerSecond / maxLinearSpeed, -1f, 1f);
        float steer = Mathf.Clamp(angularRadiansPerSecond / maxAngularSpeed, -1f, 1f);

        if (invertLinear) gas = -gas;
        if (invertAngular) steer = -steer;

        InvokeCommand(gas, steer);
    }

    public void StopRobot()
    {
        InvokeCommand(0f, 0f);
    }

    private void Awake()
    {
        CacheMethod();
    }

    private void OnValidate()
    {
        if (!Application.isPlaying)
        {
            CacheMethod();
        }
    }

    private void CacheMethod()
    {
        commandMethod = null;
        reportedMissingMethod = false;

        if (targetController == null || string.IsNullOrWhiteSpace(commandMethodName))
        {
            return;
        }

        commandMethod = targetController.GetType().GetMethod(
            commandMethodName,
            BindingFlags.Instance | BindingFlags.Public,
            null,
            new[] { typeof(float), typeof(float) },
            null);
    }

    private void InvokeCommand(float gas, float steer)
    {
        if (commandMethod == null)
        {
            CacheMethod();
        }

        if (commandMethod == null || targetController == null)
        {
            if (!reportedMissingMethod)
            {
                Debug.LogError(
                    $"GFS-X ROS1: компонент движения должен иметь public void {commandMethodName}(float, float).",
                    this);
                reportedMissingMethod = true;
            }

            return;
        }

        try
        {
            commandMethod.Invoke(targetController, new object[] { gas, steer });
        }
        catch (TargetInvocationException exception)
        {
            Exception cause = exception.InnerException ?? exception;
            Debug.LogException(cause, targetController);
        }
    }
}
