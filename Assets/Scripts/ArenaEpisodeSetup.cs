using UnityEngine;

/// <summary>
/// Concrete arena parameters for one episode. Produced either by the staged
/// success-gated curriculum (one axis changes per stage) or from the scalar
/// arena_difficulty supplied by a trainer yaml - FromScalar reproduces the
/// historical Lerp-based mapping exactly, so explicit yaml configs and the
/// standalone default behave as they always did.
/// </summary>
public readonly struct ArenaEpisodeSetup
{
    public readonly float SpawnHalfAngleDegrees;
    public readonly float MinSpawnDistance;
    public readonly float MaxSpawnDistance;
    public readonly int MinObstacles;
    public readonly int MaxObstacles;
    public readonly float DetourProbability;
    // 0..1 progress used for stats (Robot/ArenaDifficulty) and barrier scaling.
    public readonly float NormalizedDifficulty;

    public ArenaEpisodeSetup(float spawnHalfAngleDegrees, float minSpawnDistance,
        float maxSpawnDistance, int minObstacles, int maxObstacles,
        float detourProbability, float normalizedDifficulty)
    {
        SpawnHalfAngleDegrees = spawnHalfAngleDegrees;
        MinSpawnDistance = minSpawnDistance;
        MaxSpawnDistance = maxSpawnDistance;
        MinObstacles = minObstacles;
        MaxObstacles = maxObstacles;
        DetourProbability = detourProbability;
        NormalizedDifficulty = normalizedDifficulty;
    }

    public static ArenaEpisodeSetup FromScalar(float difficulty)
    {
        float d = Mathf.Clamp01(difficulty);
        int maxObstacles = Mathf.Clamp(Mathf.RoundToInt(7f * d), 0, 7);
        int minObstacles = d < 0.15f
            ? 0
            : Mathf.Clamp(Mathf.RoundToInt(4f * d), 1, Mathf.Max(1, maxObstacles));
        return new ArenaEpisodeSetup(
            // Ball only ever spawns in the +-50 deg forward cone (real-task fact),
            // so the arc caps at 50, not 180 - no behind-the-robot search.
            Mathf.Lerp(25f, 50f, d),
            Mathf.Lerp(1.1f, 1.8f, d),
            Mathf.Lerp(2.0f, 4.1f, d),
            minObstacles,
            maxObstacles,
            d >= 0.55f ? Mathf.Lerp(0.15f, 0.55f, d) : 0f,
            d);
    }
}
