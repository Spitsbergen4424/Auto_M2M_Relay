using UnityEngine;

/// <summary>
/// Single source of truth for reward magnitudes. The structure follows the
/// field-proven configuration of the original hardware project: camera-space
/// approach shaping (the same normalized signal the real YOLO pipeline emits),
/// sensor-space obstacle penalties (the only channels the physical robot can
/// perceive - it has no collision event), and Isaac-Lab-style quadratic
/// action-rate regularization to protect real motors and servos.
/// Per-step values are calibrated for ~50 calls per second
/// (TakeActionsBetweenDecisions is enabled in the scene).
/// </summary>
public static class RewardTuning
{
    // --- Approach (only while the ball is visible) ---
    // Multiplier grows from 2x (far) to 6x (touching), demanding a precise
    // final approach instead of a full-speed ram.
    public const float ApproachBaseScale = 2.0f;
    public const float ApproachProximityGain = 4.0f;
    public const float FirstDetectionReward = 0.25f;
    public const float GripperReachedReward = 0.75f;

    // --- Final approach quality (anti ball-ramming) ---
    public const float CloseBallDistance = 0.30f;  // normalized camera distance
    public const float AlignmentDistance = 0.40f;
    public const float AlignmentAngle = 0.15f;     // |HorizontalOffset| counted as centered
    public const float SlowGasMin = 0.01f;
    public const float SlowGasMax = 0.30f;
    public const float SlowApproachBonus = 0.005f;
    public const float AlignmentBonus = 0.005f;
    public const float NearBallSpeedDistance = 0.25f;
    public const float NearBallSpeedGas = 0.40f;
    public const float NearBallSpeedPenalty = 0.01f;

    // --- Blind final approach (ball vanished under the bumper) ---
    public const float BlindCrawlBonus = 0.003f;
    public const float BlindPhaseSeconds = 4f;

    // --- Detour around an occluding barrier (only when a barrier layout exists,
    // i.e. the last curriculum stages). Directed exploration: potential-based
    // progress toward the nearer free wall edge. smoke3 proved generic
    // exploration alone plateaus at ~0.24 detour success; this points the search.
    // A full ~2.5 m walk-around nets roughly +1.75, below the +5 grab. ---
    public const float DetourPathProgress = 0.7f;
    public const float MaxDetourProgressPerStep = 0.05f; // guards against teleport-like jumps

    // --- Obstacles: sensor-space only. Unity collisions stay a metric. ---
    // Magnitudes raised ~2.7x after run gfsx_cone_v1: the policy reached 93%
    // success and 0.81 detour bypass but averaged 3.1 wall scrapes/episode, so the
    // curriculum's collision cap held it on the barrier stage. Cheap contact
    // (weak penalty vs the +5 grab) let it bump its way to the ball. Stronger
    // proximity cost pushes genuinely clean navigation, which the real robot needs.
    // Thresholds unchanged: still zero cost until close, so rounding a wall edge
    // (required for the detour) stays free - only actual scraping is expensive.
    public const float SonarPenaltyThreshold = 0.15f; // normalized (0.30 m at the 2 m range)
    public const float SonarMaxPenalty = 0.08f;
    // Side IR: the policy's observation stays binary at the sensor's native 15 cm
    // (hardware-faithful), but the training-time penalty uses the simulator's ray
    // distance and starts only at half range - plain detection costs nothing,
    // squeezing closer gets progressively expensive.
    public const float SideIrPenaltyThreshold = 0.5f; // fraction of the IR range (7.5 cm)
    public const float SideIrMaxPenalty = 0.03f;

    // --- Reverse driving (the rear has no sensors to see what it hits) ---
    public const float ReverseGasThreshold = -0.1f;
    public const float ReversePenalty = 0.005f;
    public const float ReverseEscapeClearance = 0.25f; // reversing off a near wall is legitimate

    // --- Regularization ---
    // Measured in run smoke1: at 0.05 the quadratic cost of PPO's own exploration
    // noise (~sigma 0.4 per action) reached ~-5 per episode, rivalling the +5
    // success reward and stalling entropy decay. 0.01 keeps the smoothness
    // pressure while letting the task signal dominate.
    public const float ActionRatePenalty = 0.01f; // x sum of squared action deltas
    public const float DecisionStepPenalty = 0.0005f;

    // --- Terminals ---
    public const float StuckPenalty = 0.5f;
    public const float StuckWindowSeconds = 4f;
    // Threshold on the SIGNED command average over the whole window. Zero-mean
    // exploration dither averages out to ~0 and no longer counts as commanding
    // (run smoke1: the old instantaneous |gas|>0.1 check terminated ~50% of all
    // episodes as false-positive "stuck"). Sustained pushing or spinning keeps a
    // large signed average and still triggers the terminal.
    public const float StuckCommandThreshold = 0.2f;
    public const float StuckMinProgressFraction = 0.2f; // of full-speed travel over the window
    public const float EpisodeTimeLimitSeconds = 60f;
    public const float TimeoutPenalty = 0.05f;
    public const float OutOfArenaPenalty = 2f;

    public static float ProximityMultiplier(float normalizedBallDistance)
    {
        return ApproachBaseScale +
               ApproachProximityGain * (1f - Mathf.Clamp01(normalizedBallDistance));
    }

    public static float SonarProximityPenalty(float normalizedSonarDistance)
    {
        if (normalizedSonarDistance >= SonarPenaltyThreshold)
        {
            return 0f;
        }

        return SonarMaxPenalty *
               (1f - Mathf.Clamp01(normalizedSonarDistance) / SonarPenaltyThreshold);
    }

    public static float SideIrProximityPenalty(float normalizedIrDistance)
    {
        if (normalizedIrDistance >= SideIrPenaltyThreshold)
        {
            return 0f;
        }

        return SideIrMaxPenalty *
               (1f - Mathf.Clamp01(normalizedIrDistance) / SideIrPenaltyThreshold);
    }
}
