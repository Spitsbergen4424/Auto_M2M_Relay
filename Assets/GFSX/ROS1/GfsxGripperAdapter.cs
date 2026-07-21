using System;
using System.Reflection;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class GfsxGripperAdapter : MonoBehaviour
{
    [SerializeField] private Component targetController;
    [SerializeField] private string commandMethodName = "ApplyCommand";
    [SerializeField] private string hasBallPropertyName = "HasBall";
    [SerializeField] private int grabCommand = 1;
    [SerializeField] private int releaseCommand = 2;

    private MethodInfo commandMethod;
    private PropertyInfo hasBallProperty;

    public Component TargetController => targetController;

    public void Configure(Component controller)
    {
        targetController = controller;
        CacheMembers();
    }

    public void ApplyClosedCommand(bool close)
    {
        if (commandMethod == null)
        {
            CacheMembers();
        }

        if (targetController == null || commandMethod == null)
        {
            Debug.LogWarning(
                $"GFS-X ROS1: хваталка не настроена. Ожидается public void {commandMethodName}(int).",
                this);
            return;
        }

        try
        {
            commandMethod.Invoke(targetController, new object[] { close ? grabCommand : releaseCommand });
        }
        catch (TargetInvocationException exception)
        {
            Exception cause = exception.InnerException ?? exception;
            Debug.LogException(cause, targetController);
        }
    }

    public bool TryGetHasBall(out bool hasBall)
    {
        hasBall = false;
        if (hasBallProperty == null)
        {
            CacheMembers();
        }

        if (targetController == null || hasBallProperty == null)
        {
            return false;
        }

        object value = hasBallProperty.GetValue(targetController);
        if (value is bool boolValue)
        {
            hasBall = boolValue;
            return true;
        }

        return false;
    }

    private void Awake()
    {
        CacheMembers();
    }

    private void CacheMembers()
    {
        commandMethod = null;
        hasBallProperty = null;

        if (targetController == null)
        {
            return;
        }

        Type type = targetController.GetType();
        commandMethod = type.GetMethod(
            commandMethodName,
            BindingFlags.Instance | BindingFlags.Public,
            null,
            new[] { typeof(int) },
            null);
        hasBallProperty = type.GetProperty(hasBallPropertyName, BindingFlags.Instance | BindingFlags.Public);
    }
}
