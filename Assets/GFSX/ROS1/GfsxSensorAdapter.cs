using System;
using System.Reflection;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class GfsxSensorAdapter : MonoBehaviour
{
    [Header("Controller compatibility")]
    [SerializeField] private Component targetSensors;
    [SerializeField] private string ultrasonicPropertyName = "Ultrasonic01";
    [SerializeField] private string leftIrPropertyName = "LeftIr";
    [SerializeField] private string rightIrPropertyName = "RightIr";
    [SerializeField] private string gripperIrPropertyName = "GripperIr";

    [Header("Ultrasonic conversion")]
    [Tooltip("В текущем VirtualSensors 1 означает максимальную дальность, 0 — препятствие вплотную.")]
    [SerializeField] private bool oneMeansMaximumDistance = true;
    [SerializeField, Min(0.001f)] private float ultrasonicMaxDistanceMeters = 1.2f;

    private PropertyInfo ultrasonicProperty;
    private PropertyInfo leftIrProperty;
    private PropertyInfo rightIrProperty;
    private PropertyInfo gripperIrProperty;

    public Component TargetSensors => targetSensors;
    public float UltrasonicMaxDistanceMeters => ultrasonicMaxDistanceMeters;

    public void Configure(Component sensors)
    {
        targetSensors = sensors;
        CacheProperties();
    }

    public bool TryGetUltrasonicMeters(out float distanceMeters)
    {
        distanceMeters = ultrasonicMaxDistanceMeters;
        if (!TryReadFloat(ultrasonicProperty, out float normalized))
        {
            CacheProperties();
            if (!TryReadFloat(ultrasonicProperty, out normalized))
            {
                return false;
            }
        }

        normalized = Mathf.Clamp01(normalized);
        if (!oneMeansMaximumDistance)
        {
            normalized = 1f - normalized;
        }

        distanceMeters = normalized * ultrasonicMaxDistanceMeters;
        return true;
    }

    public bool TryGetLeftIr(out bool detected)
    {
        if (leftIrProperty == null) CacheProperties();
        return TryReadBoolLike(leftIrProperty, out detected);
    }

    public bool TryGetRightIr(out bool detected)
    {
        if (rightIrProperty == null) CacheProperties();
        return TryReadBoolLike(rightIrProperty, out detected);
    }

    public bool TryGetGripperIr(out bool detected)
    {
        if (gripperIrProperty == null) CacheProperties();
        return TryReadBoolLike(gripperIrProperty, out detected);
    }

    private void Awake()
    {
        CacheProperties();
    }

    private void CacheProperties()
    {
        ultrasonicProperty = null;
        leftIrProperty = null;
        rightIrProperty = null;
        gripperIrProperty = null;

        if (targetSensors == null)
        {
            return;
        }

        Type type = targetSensors.GetType();
        const BindingFlags flags = BindingFlags.Instance | BindingFlags.Public;
        ultrasonicProperty = type.GetProperty(ultrasonicPropertyName, flags);
        leftIrProperty = type.GetProperty(leftIrPropertyName, flags);
        rightIrProperty = type.GetProperty(rightIrPropertyName, flags);
        gripperIrProperty = type.GetProperty(gripperIrPropertyName, flags);
    }

    private bool TryReadBoolLike(PropertyInfo property, out bool detected)
    {
        detected = false;
        if (!TryReadFloat(property, out float value))
        {
            return false;
        }

        detected = value >= 0.5f;
        return true;
    }

    private bool TryReadFloat(PropertyInfo property, out float value)
    {
        value = 0f;
        if (targetSensors == null || property == null)
        {
            return false;
        }

        object raw = property.GetValue(targetSensors);
        if (raw == null)
        {
            return false;
        }

        try
        {
            value = Convert.ToSingle(raw);
            return true;
        }
        catch (Exception)
        {
            return false;
        }
    }
}
