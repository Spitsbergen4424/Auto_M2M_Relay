public static class ActiveSearchRewardShaping
{
    // Blind-search coverage: one small reward per newly visited grid cell is the
    // single exploration signal. Sector scans, moving viewpoints and spin
    // penalties were removed in the reward simplification - degenerate
    // behaviours (spinning in place, circling) are now cut off by the stuck
    // terminal in RobotBrain instead of per-step micro-penalties.
    public const float SearchCellSize = 0.8f;
    public const float NewAreaReward = 0.02f;
}
