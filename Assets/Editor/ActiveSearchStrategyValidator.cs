using System;
using UnityEngine;

public static class ActiveSearchStrategyValidator
{
    // Reward calls run every physics tick because TakeActionsBetweenDecisions
    // is enabled in the scene.
    private const float StepsPerSecond = 50f;

    [UnityEditor.MenuItem("Tools/URFU/Validate Reward Tuning")]
    public static void Validate()
    {
        // Approach multiplier: 2x far, 6x touching, monotonically increasing.
        RequireApproximately(RewardTuning.ProximityMultiplier(1f), 2f,
            "A distant ball must use the base approach multiplier.");
        RequireApproximately(RewardTuning.ProximityMultiplier(0f), 6f,
            "A touching-distance ball must use the maximum approach multiplier.");
        Require(RewardTuning.ProximityMultiplier(0.2f) > RewardTuning.ProximityMultiplier(0.8f),
            "The approach multiplier must grow as the ball gets closer.");

        // Sonar gradient: zero at the threshold, maximal at contact.
        RequireApproximately(
            RewardTuning.SonarProximityPenalty(RewardTuning.SonarPenaltyThreshold), 0f,
            "The sonar penalty must vanish at the threshold distance.");
        RequireApproximately(
            RewardTuning.SonarProximityPenalty(0f), RewardTuning.SonarMaxPenalty,
            "The sonar penalty must peak at contact.");
        Require(RewardTuning.SonarProximityPenalty(RewardTuning.SonarPenaltyThreshold * 0.5f) >
                RewardTuning.SonarProximityPenalty(RewardTuning.SonarPenaltyThreshold * 0.9f),
            "The sonar penalty must grow as the wall gets closer.");
        Require(RewardTuning.SonarProximityPenalty(1f) == 0f,
            "A clear sonar reading must cost nothing.");

        // Side IR gradient: mere detection is free, cost ramps inside half range.
        RequireApproximately(
            RewardTuning.SideIrProximityPenalty(RewardTuning.SideIrPenaltyThreshold), 0f,
            "The side-IR penalty must vanish at its threshold - detection alone is free.");
        RequireApproximately(RewardTuning.SideIrProximityPenalty(1f), 0f,
            "A clear side-IR reading must cost nothing.");
        RequireApproximately(
            RewardTuning.SideIrProximityPenalty(0f), RewardTuning.SideIrMaxPenalty,
            "The side-IR penalty must peak at contact.");
        Require(RewardTuning.SideIrPenaltyThreshold < 1f,
            "The side-IR penalty must start strictly inside the sensor range.");

        // Farming bounds: no per-step bonus stream may rival the terminal grab.
        float blindCrawlCeiling = RewardTuning.BlindCrawlBonus * StepsPerSecond *
                                  RewardTuning.BlindPhaseSeconds;
        Require(blindCrawlCeiling < RewardTuning.GripperReachedReward + 1f,
            "The blind-crawl window must stay worth less than actually reaching the ball.");
        float approachBonusPerSecond = (RewardTuning.SlowApproachBonus +
                                        RewardTuning.AlignmentBonus) * StepsPerSecond;
        Require(approachBonusPerSecond * RewardTuning.EpisodeTimeLimitSeconds * 0.1f < 5f,
            "Final-approach bonuses must not out-earn the success reward over a realistic approach.");

        // A wall-press must out-cost everything the robot could earn while pressed.
        float wallPressPerSecond = RewardTuning.SonarMaxPenalty * StepsPerSecond;
        Require(wallPressPerSecond > approachBonusPerSecond,
            "Pressing a wall must cost more than any bonus stream earns.");

        // The stuck terminal must dominate the exploration income of the window.
        float windowExplorationCeiling = 6f * ActiveSearchRewardShaping.NewAreaReward;
        Require(RewardTuning.StuckPenalty > windowExplorationCeiling,
            "Ending an episode stuck must outweigh the cells discoverable in one window.");

        // Detour shaping is auto-scoped: no barrier layout means no reward gradient,
        // so obstacle-free early stages are untouched by it.
        var detourlessRandomizer = new GameObject("__detour_probe").AddComponent<ArenaObstacleRandomizer>();
        try
        {
            Require(!detourlessRandomizer.TryGetDetourPathPotential(Vector3.zero, out float probe) &&
                    probe == 0f,
                "Detour potential must be unavailable without a barrier layout.");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(detourlessRandomizer.gameObject);
        }

        // Staged curriculum ladder: starts trivial, each axis only ever ramps up.
        ArenaEpisodeSetup first = SuccessGatedCurriculum.GetStageSetup(0);
        Require(first.MaxObstacles == 0 && first.DetourProbability == 0f,
            "The first stage must contain no obstacles and no barriers.");
        Require(SuccessGatedCurriculum.GetStageSetup(SuccessGatedCurriculum.StageCount - 1)
                    .DetourProbability > 0f,
            "The final stage must include occluding barriers (occlusion is part of the task).");
        Require(first.SpawnHalfAngleDegrees <= 30f,
            "The first stage must spawn the ball inside the visible cone.");
        for (int i = 0; i < SuccessGatedCurriculum.StageCount; i++)
        {
            ArenaEpisodeSetup stage = SuccessGatedCurriculum.GetStageSetup(i);
            Require(stage.MinSpawnDistance <= stage.MaxSpawnDistance,
                $"Stage {i}: spawn distance range must be valid.");
            Require(stage.MinObstacles <= stage.MaxObstacles,
                $"Stage {i}: obstacle range must be valid.");
            if (i == 0)
            {
                continue;
            }

            ArenaEpisodeSetup previous = SuccessGatedCurriculum.GetStageSetup(i - 1);
            Require(stage.SpawnHalfAngleDegrees >= previous.SpawnHalfAngleDegrees,
                $"Stage {i}: the spawn arc must never shrink.");
            Require(stage.MaxObstacles >= previous.MaxObstacles,
                $"Stage {i}: the obstacle ceiling must never shrink.");
            Require(stage.DetourProbability >= previous.DetourProbability,
                $"Stage {i}: barrier frequency must never shrink.");
            Require(stage.NormalizedDifficulty > previous.NormalizedDifficulty,
                $"Stage {i}: reported difficulty must strictly increase.");
        }

        Debug.Log("REWARD_TUNING PASSED|" +
                  $"blindCrawlCeiling={blindCrawlCeiling:F3}|" +
                  $"wallPressPerSecond={wallPressPerSecond:F3}|" +
                  $"approachBonusPerSecond={approachBonusPerSecond:F3}");
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new InvalidOperationException(message);
        }
    }

    private static void RequireApproximately(float actual, float expected, string message)
    {
        if (Mathf.Abs(actual - expected) > 0.0001f)
        {
            throw new InvalidOperationException($"{message} (expected {expected}, got {actual})");
        }
    }
}
