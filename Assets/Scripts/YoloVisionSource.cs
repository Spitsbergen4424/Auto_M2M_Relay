using UnityEngine;

/// <summary>
/// A stable vision contract shared by simulated and real YOLO sources.
/// RobotBrain depends on this contract, so switching sources does not change
/// the order or meaning of PPO observations or search-sector accounting.
/// </summary>
public abstract class YoloVisionSource : MonoBehaviour
{
    public abstract bool IsVisible { get; protected set; }
    public abstract float HorizontalOffset { get; protected set; }
    public abstract float NormalizedDistance { get; protected set; }
    public abstract float LastKnownDirection { get; protected set; }
    public abstract float TimeSinceDetection { get; protected set; }
    public virtual int WorldViewSector => 0;
}
