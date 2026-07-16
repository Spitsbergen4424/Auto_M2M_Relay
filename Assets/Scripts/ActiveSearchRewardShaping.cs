using UnityEngine;

public static class ActiveSearchRewardShaping
{
    public const float InitialScanDuration = 1.5f;
    public const float SearchCellSize = 1.2f;
    public const float MinimumMovingSpeed = 0.12f;
    public const float SpinAngularSpeed = 0.5f;
    public const float MinimumWindowDisplacement = 0.18f;
    public const float StuckWindowDuration = 2f;

    public const float NewAreaReward = 0.01f;
    public const float InitialSectorReward = 0.0005f;
    public const float MovingViewpointReward = 0.0002f;
    public const float StationarySpinPenalty = 0.001f;
    public const float StuckPenalty = 0.02f;

    public static bool ShouldPenalizeStationarySpin(
        float timeWithoutBall,
        float planarSpeed,
        float angularSpeed)
    {
        return timeWithoutBall > InitialScanDuration &&
               planarSpeed < MinimumMovingSpeed &&
               angularSpeed >= SpinAngularSpeed;
    }

    public static int ViewpointKey(Vector2Int cell, int cameraSector)
    {
        unchecked
        {
            return (cell.x * 73856093) ^ (cell.y * 19349663) ^ cameraSector;
        }
    }
}
