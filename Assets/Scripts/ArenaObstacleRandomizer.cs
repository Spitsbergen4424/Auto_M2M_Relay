using System;
using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;

public sealed class ArenaObstacleRandomizer : MonoBehaviour
{
    [SerializeField] private Transform arena;
    [SerializeField] private Transform robot;
    [SerializeField] private Transform targetBall;
    [SerializeField] private Transform obstacleContainer;
    // Pool size only; the per-episode obstacle count now comes from ArenaEpisodeSetup.
    [SerializeField, Min(1)] private int maximumObstacleCount = 7;
    [SerializeField] private float placementHalfExtent = 4.15f;
    [SerializeField] private float safeRadiusAroundRobot = 1.35f;
    [SerializeField] private float safeRadiusAroundBall = 1.15f;
    [SerializeField] private int randomSeed = 1;

    private readonly List<Transform> obstacles = new List<Transform>();
    private int generation;
    private Vector3 detourCenterLocal;
    private Vector3 detourDirectionLocal;
    private Vector3 detourLeftEntryLocal;
    private Vector3 detourLeftExitLocal;
    private Vector3 detourRightEntryLocal;
    private Vector3 detourRightExitLocal;
    private float detourHalfDepth;

    public float CurrentDifficulty { get; private set; } = 1f;
    public bool RequiresDetour => CurrentDifficulty >= 0.55f && randomSeed % 2 == 0;
    public bool HasDetourLayout { get; private set; }

    // Shortest walk-around distance from a world position to the (ground-truth)
    // ball, routing through whichever free wall edge is nearer. Reward-only
    // privileged geometry: the policy never sees this, it only feels the reward
    // gradient. Lower potential = closer to being around the barrier.
    public bool TryGetDetourPathPotential(Vector3 worldPosition, out float potential)
    {
        potential = 0f;
        if (!HasDetourLayout || arena == null || targetBall == null)
        {
            return false;
        }

        Vector3 current = arena.InverseTransformPoint(worldPosition);
        Vector3 goal = arena.InverseTransformPoint(targetBall.position);
        current.y = 0f;
        goal.y = 0f;
        float longitudinal = Vector3.Dot(current - detourCenterLocal, detourDirectionLocal);

        if (longitudinal < -detourHalfDepth)
        {
            // Still in front of the barrier: reach an edge, pass it, then the goal.
            float left = HorizontalDistance(current, detourLeftEntryLocal) +
                         HorizontalDistance(detourLeftEntryLocal, detourLeftExitLocal) +
                         HorizontalDistance(detourLeftExitLocal, goal);
            float right = HorizontalDistance(current, detourRightEntryLocal) +
                          HorizontalDistance(detourRightEntryLocal, detourRightExitLocal) +
                          HorizontalDistance(detourRightExitLocal, goal);
            potential = Mathf.Min(left, right);
        }
        else if (longitudinal <= detourHalfDepth)
        {
            // Alongside the barrier: clear the near exit, then head to the goal.
            potential = Mathf.Min(
                HorizontalDistance(current, detourLeftExitLocal) + HorizontalDistance(detourLeftExitLocal, goal),
                HorizontalDistance(current, detourRightExitLocal) + HorizontalDistance(detourRightExitLocal, goal));
        }
        else
        {
            // Past the barrier: a straight shot remains.
            potential = HorizontalDistance(current, goal);
        }

        return true;
    }

    public void Configure(Transform arenaTransform, Transform robotTransform, Transform ballTransform, int seed)
    {
        arena = arenaTransform;
        robot = robotTransform;
        targetBall = ballTransform;
        randomSeed = seed;
        CacheObstacles();
    }

    // The full episode setup comes from RobotBrain (single per-episode source:
    // yaml scalar, staged curriculum, or the standalone default) so obstacle
    // counts and barrier frequency always match the ball spawn of the episode.
    public void RandomizeLayout(ArenaEpisodeSetup setup)
    {
        if (arena == null)
        {
            arena = FindChild(transform, "TrainingArena");
        }

        if (arena == null)
        {
            return;
        }

        EnsureObstaclePool();
        CurrentDifficulty = setup.NormalizedDifficulty;
        var random = new System.Random(unchecked(randomSeed * 7919 + generation++ * 104729));
        int poolMaximum = Mathf.Clamp(setup.MaxObstacles, 0, obstacles.Count);
        int poolMinimum = Mathf.Clamp(setup.MinObstacles, 0, poolMaximum);
        int count = poolMaximum > poolMinimum
            ? random.Next(poolMinimum, poolMaximum + 1)
            : poolMaximum;
        var occupied = new List<Vector3>(count);

        Vector3 robotLocal = robot != null ? arena.InverseTransformPoint(robot.position) : Vector3.zero;
        Vector3 ballLocal = targetBall != null ? arena.InverseTransformPoint(targetBall.position) : new Vector3(0f, 0f, 2f);
        float floorTop = FindFloorTop();
        // An explicit detour_probability environment parameter still overrides
        // the per-stage value (used by evaluation configs).
        float detourOverride = Academy.Instance.EnvironmentParameters.GetWithDefault(
            "detour_probability", -1f);
        float barrierProbability = detourOverride >= 0f
            ? Mathf.Clamp01(detourOverride)
            : setup.DetourProbability;
        bool createDetour = barrierProbability > 0f && random.NextDouble() < barrierProbability;
        if (createDetour)
        {
            count = Mathf.Max(1, count);
        }
        HasDetourLayout = PlaceDetourBarrier(robotLocal, ballLocal, floorTop, occupied, createDetour);
        int firstRandomObstacle = HasDetourLayout ? 1 : 0;

        for (int index = 0; index < obstacles.Count; index++)
        {
            Transform obstacle = obstacles[index];
            bool active = index < count;
            obstacle.gameObject.SetActive(active);
            if (!active || index < firstRandomObstacle)
            {
                continue;
            }

            float width = NextFloat(random, 0.55f, 1.25f);
            float depth = NextFloat(random, 0.55f, 1.25f);
            float height = NextFloat(random, 0.55f, 1.35f);
            float clearance = Mathf.Max(width, depth) * 0.65f + 0.45f;
            Vector3 position = Vector3.zero;

            for (int attempt = 0; attempt < 100; attempt++)
            {
                position = new Vector3(
                    NextFloat(random, -placementHalfExtent, placementHalfExtent),
                    floorTop + height * 0.5f,
                    NextFloat(random, -placementHalfExtent, placementHalfExtent));

                if (HorizontalDistance(position, robotLocal) < safeRadiusAroundRobot + clearance ||
                    HorizontalDistance(position, ballLocal) < safeRadiusAroundBall + clearance)
                {
                    continue;
                }

                bool overlaps = false;
                foreach (Vector3 other in occupied)
                {
                    if (HorizontalDistance(position, other) < clearance + other.y)
                    {
                        overlaps = true;
                        break;
                    }
                }

                if (!overlaps)
                {
                    break;
                }
            }

            obstacle.localPosition = position;
            obstacle.localRotation = Quaternion.Euler(0f, NextFloat(random, 0f, 360f), 0f);
            obstacle.localScale = new Vector3(width, height, depth);
            occupied.Add(new Vector3(position.x, clearance, position.z));
        }

        EnsureReachablePath(robotLocal, ballLocal, firstRandomObstacle);
    }

    private void EnsureReachablePath(Vector3 robotLocal, Vector3 ballLocal, int protectedObstacleCount)
    {
        // Remove only optional random obstacles until a grid path exists. The deliberate
        // detour barrier is protected, so evaluation still requires obstacle avoidance.
        for (int index = obstacles.Count - 1; index >= protectedObstacleCount && !HasReachablePath(robotLocal, ballLocal); index--)
        {
            obstacles[index].gameObject.SetActive(false);
        }

        if (!HasReachablePath(robotLocal, ballLocal))
        {
            Debug.LogWarning($"No reachable path generated in {name}; keeping the safest available layout.", this);
        }
    }

    private bool HasReachablePath(Vector3 robotLocal, Vector3 ballLocal)
    {
        const int gridSize = 25;
        float cellSize = placementHalfExtent * 2f / (gridSize - 1);
        bool[,] blocked = new bool[gridSize, gridSize];
        foreach (Transform obstacle in obstacles)
        {
            if (!obstacle.gameObject.activeInHierarchy)
            {
                continue;
            }

            Vector3 local = obstacle.localPosition;
            float radius = Mathf.Sqrt(obstacle.localScale.x * obstacle.localScale.x +
                                      obstacle.localScale.z * obstacle.localScale.z) * 0.5f + 0.4f;
            for (int x = 0; x < gridSize; x++)
            {
                for (int z = 0; z < gridSize; z++)
                {
                    Vector2 point = new Vector2(-placementHalfExtent + x * cellSize,
                        -placementHalfExtent + z * cellSize);
                    if (Vector2.Distance(point, new Vector2(local.x, local.z)) <= radius)
                    {
                        blocked[x, z] = true;
                    }
                }
            }
        }

        Vector2Int start = ToGrid(robotLocal, gridSize, cellSize);
        Vector2Int goal = ToGrid(ballLocal, gridSize, cellSize);
        blocked[start.x, start.y] = false;
        blocked[goal.x, goal.y] = false;
        bool[,] visited = new bool[gridSize, gridSize];
        var queue = new Queue<Vector2Int>();
        queue.Enqueue(start);
        visited[start.x, start.y] = true;
        Vector2Int[] directions =
        {
            Vector2Int.left, Vector2Int.right, Vector2Int.up, Vector2Int.down
        };

        while (queue.Count > 0)
        {
            Vector2Int cell = queue.Dequeue();
            if (cell == goal)
            {
                return true;
            }

            foreach (Vector2Int direction in directions)
            {
                Vector2Int next = cell + direction;
                if (next.x < 0 || next.y < 0 || next.x >= gridSize || next.y >= gridSize ||
                    visited[next.x, next.y] || blocked[next.x, next.y])
                {
                    continue;
                }

                visited[next.x, next.y] = true;
                queue.Enqueue(next);
            }
        }

        return false;
    }

    private Vector2Int ToGrid(Vector3 local, int gridSize, float cellSize)
    {
        int x = Mathf.Clamp(Mathf.RoundToInt((local.x + placementHalfExtent) / cellSize), 0, gridSize - 1);
        int z = Mathf.Clamp(Mathf.RoundToInt((local.z + placementHalfExtent) / cellSize), 0, gridSize - 1);
        return new Vector2Int(x, z);
    }

    private bool PlaceDetourBarrier(Vector3 robotLocal, Vector3 ballLocal, float floorTop,
        List<Vector3> occupied, bool createDetour)
    {
        if (!createDetour || obstacles.Count == 0)
        {
            HasDetourLayout = false;
            return false;
        }

        Vector3 direction = ballLocal - robotLocal;
        direction.y = 0f;
        float directDistance = direction.magnitude;
        if (directDistance < 0.8f)
        {
            return false;
        }

        direction /= directDistance;
        Vector3 perpendicular = new Vector3(-direction.z, 0f, direction.x);
        float requestedWidth = Mathf.Lerp(1.25f, 3.0f, CurrentDifficulty);
        float barrierWidth = Mathf.Clamp(requestedWidth, 1.1f, placementHalfExtent * 1.2f);
        float barrierDepth = 0.65f;
        // The barrier must occlude the camera ray, not merely block the tracks.
        // A lower wall allowed the policy to see the ball over its top and inflated
        // the apparent detour success rate without teaching a real bypass.
        float barrierHeight = 1.60f;
        Vector3 position = Vector3.Lerp(robotLocal, ballLocal, 0.5f);
        position.y = floorTop + barrierHeight * 0.5f;

        Transform barrier = obstacles[0];
        barrier.gameObject.SetActive(true);
        barrier.name = "Obstacle_01_DetourBarrier";
        barrier.localPosition = position;
        float yaw = Mathf.Atan2(-perpendicular.z, perpendicular.x) * Mathf.Rad2Deg;
        barrier.localRotation = Quaternion.Euler(0f, yaw, 0f);
        barrier.localScale = new Vector3(barrierWidth, barrierHeight, barrierDepth);
        occupied.Add(new Vector3(position.x, barrierWidth * 0.55f + 0.35f, position.z));

        // Navigation waypoints for the detour path potential: the two free edges
        // of the wall, each with an entry (near side) and exit (far side) point.
        float sideClearance = barrierWidth * 0.5f + 0.75f;
        detourHalfDepth = barrierDepth * 0.5f + 0.55f;
        detourDirectionLocal = direction;
        Vector3 flatCenter = new Vector3(position.x, 0f, position.z);
        detourCenterLocal = flatCenter;
        detourLeftEntryLocal = flatCenter + perpendicular * sideClearance - direction * detourHalfDepth;
        detourLeftExitLocal = flatCenter + perpendicular * sideClearance + direction * detourHalfDepth;
        detourRightEntryLocal = flatCenter - perpendicular * sideClearance - direction * detourHalfDepth;
        detourRightExitLocal = flatCenter - perpendicular * sideClearance + direction * detourHalfDepth;
        return true;
    }

    private void EnsureObstaclePool()
    {
        if (obstacleContainer == null)
        {
            Transform existing = arena.Find("RandomObstacles");
            if (existing != null)
            {
                obstacleContainer = existing;
            }
            else
            {
                var container = new GameObject("RandomObstacles");
                obstacleContainer = container.transform;
                obstacleContainer.SetParent(arena, false);
            }
        }

        CacheObstacles();
        while (obstacles.Count < maximumObstacleCount)
        {
            GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
            obstacle.name = $"Obstacle_{obstacles.Count + 1:00}";
            obstacle.transform.SetParent(obstacleContainer, false);
            obstacles.Add(obstacle.transform);
        }
    }

    private void CacheObstacles()
    {
        obstacles.Clear();
        if (obstacleContainer == null)
        {
            return;
        }

        for (int index = 0; index < obstacleContainer.childCount; index++)
        {
            obstacles.Add(obstacleContainer.GetChild(index));
        }
    }

    private float FindFloorTop()
    {
        Transform floor = FindChild(arena, "Floor");
        return floor != null ? floor.localPosition.y + floor.localScale.y * 0.5f : 0f;
    }

    private static Transform FindChild(Transform root, string childName)
    {
        if (root == null)
        {
            return null;
        }

        foreach (Transform item in root.GetComponentsInChildren<Transform>(true))
        {
            if (item.name.Equals(childName, StringComparison.OrdinalIgnoreCase))
            {
                return item;
            }
        }

        return null;
    }

    private static float NextFloat(System.Random random, float minimum, float maximum)
    {
        return minimum + (float)random.NextDouble() * (maximum - minimum);
    }

    private static float HorizontalDistance(Vector3 first, Vector3 second)
    {
        return Vector2.Distance(new Vector2(first.x, first.z), new Vector2(second.x, second.z));
    }
}
