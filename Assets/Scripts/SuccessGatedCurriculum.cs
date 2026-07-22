using UnityEngine;

/// <summary>
/// Environment-side staged curriculum. Two rules distinguish it from the old
/// reward-threshold lessons and from the first success-gate:
///
/// 1. One axis per stage. The scalar difficulty used to widen the spawn arc,
///    add obstacles and introduce occluding barriers all at once (the 0.4->0.7
///    jump in run smoke3 dropped success from 0.75 to 0.35). Here every stage
///    changes a single thing: cone width, then distance, then obstacles, then
///    barrier frequency. The real task places the ball only in a +-50 deg
///    forward cone, so no stage searches behind the robot - the job is to drive
///    forward and route around whatever blocks the way ahead.
///
/// 2. Plateau gating. smoke3 also showed stages advancing while the policy was
///    still improving. A stage is now left only when the policy has BOTH
///    stopped improving (no evaluation block beats the stage's best by more
///    than PlateauEpsilon for PlateauBlocksRequired consecutive blocks) AND
///    demonstrated competence (success rate and collision caps over the block).
///
/// Shared by all robots in the scene; episodes pool into one evaluation window.
/// </summary>
public static class SuccessGatedCurriculum
{
    public const int EvaluationWindowEpisodes = 200;
    public const float MinSuccessToAdvance = 0.65f;
    // Relaxed 1.0 -> 1.5 alongside the stronger sonar/IR penalties (RewardTuning):
    // squeezing past an occluding wall can legitimately brush it once, so an
    // impossible-to-clear cap would stall the barrier stages even for a tidy
    // policy. The penalty increase is what actually drives collisions down; this
    // just gives the gate realistic slack.
    public const float MaxMeanCollisions = 1.5f;
    public const float PlateauEpsilon = 0.03f;
    public const int PlateauBlocksRequired = 2;
    public const int MinBlocksAtStage = 3;

    public readonly struct Stage
    {
        public readonly string Name;
        public readonly ArenaEpisodeSetup Setup;

        public Stage(string name, ArenaEpisodeSetup setup)
        {
            Name = name;
            Setup = setup;
        }
    }

    private static readonly Stage[] Stages = CreateStages();

    private static Stage[] CreateStages()
    {
        // name, spawn half-angle, min/max spawn distance, min/max obstacles, barrier probability.
        // Spawn arc caps at 50 deg: the real ball never appears outside the forward cone.
        (string name, float arc, float minDist, float maxDist, int minObs, int maxObs, float detour)[] spec =
        {
            ("Centered",        25f, 1.1f, 2.0f, 0, 0, 0f),   // baseline: nearly ahead, close
            ("ForwardCone",     50f, 1.1f, 2.5f, 0, 0, 0f),   // axis: full +-50 cone
            ("ForwardFar",      50f, 1.5f, 4.1f, 0, 0, 0f),   // axis: distance
            ("FewObstacles",    50f, 1.5f, 4.1f, 1, 3, 0f),   // axis: obstacles
            ("ManyObstacles",   50f, 1.5f, 4.1f, 4, 7, 0f),   // axis: obstacles (full)
            ("RareBarrier",     50f, 1.5f, 4.1f, 4, 7, 0.25f),// axis: occluding barrier
            ("FrequentBarrier", 50f, 1.5f, 4.1f, 4, 7, 0.5f), // axis: barrier frequency
        };

        var stages = new Stage[spec.Length];
        for (int i = 0; i < spec.Length; i++)
        {
            float progress = spec.Length > 1 ? (float)i / (spec.Length - 1) : 1f;
            (string name, float arc, float minDist, float maxDist, int minObs, int maxObs, float detour) = spec[i];
            stages[i] = new Stage(name, new ArenaEpisodeSetup(
                arc, minDist, maxDist, minObs, maxObs, detour, progress));
        }

        return stages;
    }

    private static int stageIndex;
    private static int windowEpisodes;
    private static int windowSuccesses;
    private static long windowCollisions;
    private static int blocksAtStage;
    private static int plateauBlocks;
    private static float bestBlockSuccess;

    public static ArenaEpisodeSetup CurrentSetup => Stages[stageIndex].Setup;
    public static string CurrentStageName => Stages[stageIndex].Name;
    public static int StageCount => Stages.Length;

    public static ArenaEpisodeSetup GetStageSetup(int index)
    {
        return Stages[Mathf.Clamp(index, 0, Stages.Length - 1)].Setup;
    }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void ResetState()
    {
        stageIndex = 0;
        ResetStageTracking();
    }

    private static void ResetStageTracking()
    {
        windowEpisodes = 0;
        windowSuccesses = 0;
        windowCollisions = 0;
        blocksAtStage = 0;
        plateauBlocks = 0;
        bestBlockSuccess = 0f;
    }

    public static void ReportEpisode(bool success, int collisionCount)
    {
        if (stageIndex >= Stages.Length - 1)
        {
            return;
        }

        windowEpisodes++;
        if (success)
        {
            windowSuccesses++;
        }

        windowCollisions += collisionCount;
        if (windowEpisodes < EvaluationWindowEpisodes)
        {
            return;
        }

        float successRate = (float)windowSuccesses / windowEpisodes;
        float meanCollisions = (float)windowCollisions / windowEpisodes;
        blocksAtStage++;

        bool improving = successRate > bestBlockSuccess + PlateauEpsilon;
        plateauBlocks = improving ? 0 : plateauBlocks + 1;
        bestBlockSuccess = Mathf.Max(bestBlockSuccess, successRate);

        bool competent = successRate >= MinSuccessToAdvance && meanCollisions <= MaxMeanCollisions;
        bool plateaued = blocksAtStage >= MinBlocksAtStage && plateauBlocks >= PlateauBlocksRequired;

        if (competent && plateaued)
        {
            stageIndex++;
            Debug.Log($"[SuccessGatedCurriculum] Stage up -> '{Stages[stageIndex].Name}' " +
                      $"({stageIndex + 1}/{Stages.Length}): plateaued at success {successRate:P0}, " +
                      $"collisions {meanCollisions:F2}, best block {bestBlockSuccess:P0}.");
            ResetStageTracking();
            return;
        }

        Debug.Log($"[SuccessGatedCurriculum] Stage '{Stages[stageIndex].Name}' block {blocksAtStage}: " +
                  $"success {successRate:P0} (best {bestBlockSuccess:P0}), collisions {meanCollisions:F2}, " +
                  $"plateau {plateauBlocks}/{PlateauBlocksRequired}" +
                  (improving ? " - still improving." : competent ? " - awaiting plateau." : " - below the bar."));
        windowEpisodes = 0;
        windowSuccesses = 0;
        windowCollisions = 0;
    }
}
