public static class RewardTuning
{
    // Task progress. Ground-truth distance is used only while the ball is visible.
    public const float VisibleDistanceProgress = 1.2f;
    public const float ImmediateDetectionReward = 0.25f;
    public const float DelayedDetectionReward = 1.50f;
    public const float DelayedDetectionSeconds = 1.50f;
    public const float DetourDiscoveryReward = 2.0f;
    public const float LostSightPenalty = 0.08f;
    public const float GripperReachedReward = 0.75f;

    // Navigation. UltrasonicNormalized is 0 at contact and 1 at maximum range.
    public const float ObstacleClearanceThreshold = 0.45f;
    public const float ObstacleClearanceProgress = 0.05f;
    // Strong simulation-only shaping for blind navigation around the barrier.
    // The policy never observes the hidden ball position: it still has to act from
    // camera/IR/range data, but useful movement toward either free wall edge must
    // dominate aimless coverage rewards during training.
    public const float DetourPathProgress = 1.50f;
    public const float MaximumDetourProgressPerDecision = 0.12f;
    public const float DetourStageReward = 1.50f;
    public const float CriticalObstacleDistance = 0.10f;
    public const float CriticalObstaclePenalty = 0.006f;
    public const float SideIrPenalty = 0.0005f;
    public const float CollisionPenalty = 0.15f;

    // Regularization. At DecisionPeriod=5 this is applied about ten times per second.
    public const float ActionChangePenalty = 0.0005f;
    public const float DecisionStepPenalty = 0.0005f;
}
