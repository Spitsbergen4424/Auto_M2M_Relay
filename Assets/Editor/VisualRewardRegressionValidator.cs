using System;
using UnityEngine;

public static class VisualRewardRegressionValidator
{
    private const float TimePenalty = -0.0004f;
    private const int EpisodeSteps = 5000;

    [UnityEditor.MenuItem("Tools/URFU/Validate Visual Rewards")]
    public static void Validate()
    {
        float idleProgress = VisualRewardShaping.CalculateProgress(0f, 0f, 1f, 1f);
        RequireApproximately(idleProgress, 0f, "Standing still must not earn visual progress.");

        float improved = VisualRewardShaping.CalculateProgress(1f, 0f, 0f, 1f);
        Require(improved > 0f, "Improved camera aim and alignment must be rewarded.");

        float worsened = VisualRewardShaping.CalculateProgress(0f, 1f, 1f, 0f);
        Require(worsened < 0f, "Worsened camera aim and alignment must be penalized.");
        RequireApproximately(improved, -worsened, "Progress rewards must be symmetric.");
        float loseAndReacquireCycle = improved - 0.01f + worsened;
        Require(loseAndReacquireCycle < 0f,
            "Losing and reacquiring the ball must not allow reward farming.");

        float oldIdleEpisodeReward = EpisodeSteps * (0.0012f + 0.0008f + TimePenalty);
        float newIdleEpisodeReward = EpisodeSteps * (idleProgress + TimePenalty);
        Require(oldIdleEpisodeReward > 5f, "Baseline must reproduce the idle exploit.");
        Require(newIdleEpisodeReward < 0f, "New shaping must make a full idle episode unprofitable.");

        Debug.Log($"VISUAL_REWARD_REGRESSION PASSED|oldIdle={oldIdleEpisodeReward:F4}|" +
                  $"newIdle={newIdleEpisodeReward:F4}|improve={improved:F4}|" +
                  $"worsen={worsened:F4}|reacquireCycle={loseAndReacquireCycle:F4}");
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
        if (!Mathf.Approximately(actual, expected))
        {
            throw new InvalidOperationException($"{message} Expected {expected}, got {actual}.");
        }
    }
}
