using System;
using UnityEngine;

public static class ActiveSearchStrategyValidator
{
    private const float TimePenalty = 0.0004f;
    private const int TenSecondSteps = 500;

    [UnityEditor.MenuItem("Tools/URFU/Validate Active Search Strategy")]
    public static void Validate()
    {
        Require(!ActiveSearchRewardShaping.ShouldPenalizeStationarySpin(1f, 0f, 1f),
            "The initial scan grace period must allow stationary rotation.");
        Require(ActiveSearchRewardShaping.ShouldPenalizeStationarySpin(2f, 0.02f, 1f),
            "Repeated stationary rotation must be penalized.");
        Require(!ActiveSearchRewardShaping.ShouldPenalizeStationarySpin(2f, 0.2f, 1f),
            "Rotation while relocating must remain allowed.");

        var origin = Vector2Int.zero;
        Require(ActiveSearchRewardShaping.ViewpointKey(origin, 0) !=
                ActiveSearchRewardShaping.ViewpointKey(origin, 1),
            "Different view sectors must have different keys.");
        Require(ActiveSearchRewardShaping.ViewpointKey(origin, 0) !=
                ActiveSearchRewardShaping.ViewpointKey(Vector2Int.right, 0),
            "The same sector in a new area must have a different key.");

        float initialScanReward = 16f * ActiveSearchRewardShaping.InitialSectorReward;
        int penalizedSpinSteps = TenSecondSteps - Mathf.CeilToInt(
            ActiveSearchRewardShaping.InitialScanDuration / 0.02f);
        float stationarySpinReward = initialScanReward - TenSecondSteps * TimePenalty -
                                     penalizedSpinSteps * ActiveSearchRewardShaping.StationarySpinPenalty -
                                     4f * ActiveSearchRewardShaping.StuckPenalty;

        float openArenaSearchReward = 7f * ActiveSearchRewardShaping.NewAreaReward +
                                      20f * ActiveSearchRewardShaping.MovingViewpointReward -
                                      TenSecondSteps * TimePenalty;
        float detourSearchReward = 5f * ActiveSearchRewardShaping.NewAreaReward +
                                   16f * ActiveSearchRewardShaping.MovingViewpointReward -
                                   TenSecondSteps * TimePenalty;
        float denseArenaSearchReward = 3f * ActiveSearchRewardShaping.NewAreaReward +
                                       12f * ActiveSearchRewardShaping.MovingViewpointReward -
                                       TenSecondSteps * TimePenalty -
                                       ActiveSearchRewardShaping.StuckPenalty;

        Require(stationarySpinReward < openArenaSearchReward,
            "Active search must be better than stationary spinning in an open arena.");
        Require(stationarySpinReward < detourSearchReward,
            "Relocating around a detour must be better than stationary spinning.");
        Require(stationarySpinReward < denseArenaSearchReward,
            "Even slow progress in a dense arena must beat stationary spinning.");
        Require(stationarySpinReward < 0f, "Stationary spinning must remain unprofitable.");

        Debug.Log($"ACTIVE_SEARCH_STRATEGY PASSED|spin10s={stationarySpinReward:F4}|" +
                  $"open10s={openArenaSearchReward:F4}|detour10s={detourSearchReward:F4}|" +
                  $"dense10s={denseArenaSearchReward:F4}");
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new InvalidOperationException(message);
        }
    }
}
