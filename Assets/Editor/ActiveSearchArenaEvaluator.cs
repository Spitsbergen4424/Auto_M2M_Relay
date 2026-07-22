using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class ActiveSearchArenaEvaluator
{
    private const string ScenePath = "Assets/Scenes/SampleScene.unity";
    private const int Generations = 3;
    private const int GridSize = 41;
    private const float HalfExtent = 4.5f;
    private const float DetectionDistance = 2f;
    private const float RobotClearance = 0.3f;
    private const float AssumedSearchSpeed = 0.5f;
    // Behavioural assumption for arena scoring (how long a robot spends looking
    // around); no longer a reward constant after the reward simplification.
    private const float AssumedScanSeconds = 1.5f;

    private sealed class Result
    {
        public string Kind;
        public bool Reachable;
        public bool VisibleAtStart;
        public float DistanceUntilDetection;
        public float ActiveSeconds;
        public float StopAndScanSeconds;
    }

    [UnityEditor.MenuItem("Tools/URFU/Evaluate Active Search Arenas")]
    public static void Evaluate()
    {
        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        ArenaObstacleRandomizer[] randomizers = UnityEngine.Object.FindObjectsByType<ArenaObstacleRandomizer>(
            FindObjectsInactive.Include, FindObjectsSortMode.None);
        var results = new List<Result>(randomizers.Length * Generations);

        for (int generation = 0; generation < Generations; generation++)
        {
            for (int arenaIndex = 0; arenaIndex < randomizers.Length; arenaIndex++)
            {
                ArenaObstacleRandomizer randomizer = randomizers[arenaIndex];
                RobotBrain brain = randomizer.GetComponentInChildren<RobotBrain>(true);
                Transform ball = FindBall(randomizer.transform);
                if (brain == null || ball == null)
                {
                    continue;
                }

                Vector3 start = brain.transform.position;
                float angle = Mathf.Repeat(arenaIndex * 137.5f + generation * 71f, 360f);
                float distance = 2.2f + ((arenaIndex + generation * 3) % 7) * 0.27f;
                Vector3 direction = Quaternion.AngleAxis(angle, Vector3.up) * brain.transform.right;
                Vector3 ballPosition = start + direction.normalized * Mathf.Min(distance, 3.82f);
                ballPosition.y = ball.position.y;
                ball.position = ballPosition;

                // Arena stress test always runs at worst-case obstacle density.
                randomizer.RandomizeLayout(ArenaEpisodeSetup.FromScalar(1f));
                Physics.SyncTransforms();
                List<Bounds> obstacles = CollectObstacleBounds(randomizer.transform);
                bool openVariant = arenaIndex % 10 == 0;
                if (openVariant)
                {
                    foreach (Transform obstacle in randomizer.transform.GetComponentsInChildren<Transform>(true)
                                 .Where(item => item.name.StartsWith("Obstacle_")))
                    {
                        obstacle.gameObject.SetActive(false);
                    }

                    obstacles.Clear();
                }

                string kind = openVariant ? "open" : randomizer.HasDetourLayout ? "detour" :
                    obstacles.Count >= 6 ? "dense" : "random";
                results.Add(EvaluateLayout(start, ball.position, obstacles, kind));
            }
        }

        Require(results.Count >= 100, $"Expected at least 100 arena trials, got {results.Count}.");
        List<Result> reachable = results.Where(item => item.Reachable).ToList();
        float reachableRate = reachable.Count * 100f / results.Count;
        Require(reachableRate >= 90f,
            $"The evaluator found paths in only {reachableRate:F1}% of generated layouts.");
        PrintGroup("all", reachable);
        foreach (IGrouping<string, Result> group in reachable.GroupBy(item => item.Kind).OrderBy(item => item.Key))
        {
            PrintGroup(group.Key, group.ToList());
        }

        float activeAverage = reachable.Average(item => item.ActiveSeconds);
        float stopAverage = reachable.Average(item => item.StopAndScanSeconds);
        Require(activeAverage < stopAverage,
            "Continuous moving search must be faster than stopping in every cell.");
        Debug.Log($"ACTIVE_SEARCH_ARENA_EVALUATION PASSED|trials={results.Count}|" +
                  $"reachable={reachableRate:F1}%|" +
                  $"activeAvg={activeAverage:F2}s|stopScanAvg={stopAverage:F2}s|" +
                  $"timeSaved={(1f - activeAverage / stopAverage) * 100f:F1}%");
    }

    private static Result EvaluateLayout(Vector3 start, Vector3 ball, List<Bounds> obstacles, string kind)
    {
        List<Vector3> path = FindPath(start, ball, obstacles);
        bool visibleAtStart = CanDetect(start, ball, obstacles);
        float distanceUntilDetection = 0f;
        bool detected = visibleAtStart;
        int visitedCells = 0;
        if (!detected && path != null)
        {
            for (int index = 1; index < path.Count; index++)
            {
                distanceUntilDetection += Vector3.Distance(path[index - 1], path[index]);
                visitedCells++;
                if (CanDetect(path[index], ball, obstacles))
                {
                    detected = true;
                    break;
                }
            }
        }

        float travelSeconds = distanceUntilDetection / AssumedSearchSpeed;
        float initialScan = visibleAtStart ? 0f : AssumedScanSeconds;
        return new Result
        {
            Kind = kind,
            Reachable = path != null && detected,
            VisibleAtStart = visibleAtStart,
            DistanceUntilDetection = distanceUntilDetection,
            ActiveSeconds = travelSeconds + initialScan,
            StopAndScanSeconds = travelSeconds + initialScan +
                                 visitedCells * AssumedScanSeconds
        };
    }

    private static List<Vector3> FindPath(Vector3 start, Vector3 goal, List<Bounds> obstacles)
    {
        float cell = HalfExtent * 2f / (GridSize - 1);
        Vector2Int startCell = ToGrid(start, start, cell);
        Vector2Int goalCell = ToGrid(goal, start, cell);
        bool[,] blocked = new bool[GridSize, GridSize];
        for (int x = 0; x < GridSize; x++)
        {
            for (int z = 0; z < GridSize; z++)
            {
                Vector3 point = ToWorld(x, z, start, cell);
                blocked[x, z] = obstacles.Any(bounds => ContainsExpanded(bounds, point, RobotClearance));
            }
        }

        blocked[startCell.x, startCell.y] = false;
        blocked[goalCell.x, goalCell.y] = false;
        var queue = new Queue<Vector2Int>();
        var previous = new Dictionary<Vector2Int, Vector2Int>();
        queue.Enqueue(startCell);
        previous[startCell] = startCell;
        Vector2Int[] directions =
        {
            Vector2Int.left, Vector2Int.right, Vector2Int.up, Vector2Int.down
        };

        while (queue.Count > 0)
        {
            Vector2Int current = queue.Dequeue();
            if (current == goalCell)
            {
                break;
            }

            foreach (Vector2Int direction in directions)
            {
                Vector2Int next = current + direction;
                if (next.x < 0 || next.y < 0 || next.x >= GridSize || next.y >= GridSize ||
                    blocked[next.x, next.y] || previous.ContainsKey(next))
                {
                    continue;
                }

                previous[next] = current;
                queue.Enqueue(next);
            }
        }

        if (!previous.ContainsKey(goalCell))
        {
            return null;
        }

        var cells = new List<Vector2Int>();
        for (Vector2Int current = goalCell;; current = previous[current])
        {
            cells.Add(current);
            if (current == startCell)
            {
                break;
            }
        }

        cells.Reverse();
        return cells.Select(item => ToWorld(item.x, item.y, start, cell)).ToList();
    }

    private static bool CanDetect(Vector3 position, Vector3 ball, List<Bounds> obstacles)
    {
        Vector2 origin = new Vector2(position.x, position.z);
        Vector2 target = new Vector2(ball.x, ball.z);
        if (Vector2.Distance(origin, target) > DetectionDistance)
        {
            return false;
        }

        foreach (Bounds bounds in obstacles)
        {
            Rect rectangle = new Rect(bounds.min.x, bounds.min.z, bounds.size.x, bounds.size.z);
            if (SegmentIntersectsRect(origin, target, rectangle))
            {
                return false;
            }
        }

        return true;
    }

    private static bool SegmentIntersectsRect(Vector2 start, Vector2 end, Rect rect)
    {
        const int samples = 30;
        for (int index = 1; index < samples; index++)
        {
            Vector2 point = Vector2.Lerp(start, end, index / (float)samples);
            if (rect.Contains(point))
            {
                return true;
            }
        }

        return false;
    }

    private static bool ContainsExpanded(Bounds bounds, Vector3 point, float expansion)
    {
        return point.x >= bounds.min.x - expansion && point.x <= bounds.max.x + expansion &&
               point.z >= bounds.min.z - expansion && point.z <= bounds.max.z + expansion;
    }

    private static Vector2Int ToGrid(Vector3 point, Vector3 center, float cell)
    {
        return new Vector2Int(
            Mathf.Clamp(Mathf.RoundToInt((point.x - center.x + HalfExtent) / cell), 0, GridSize - 1),
            Mathf.Clamp(Mathf.RoundToInt((point.z - center.z + HalfExtent) / cell), 0, GridSize - 1));
    }

    private static Vector3 ToWorld(int x, int z, Vector3 center, float cell)
    {
        return new Vector3(center.x - HalfExtent + x * cell, center.y,
            center.z - HalfExtent + z * cell);
    }

    private static List<Bounds> CollectObstacleBounds(Transform arenaRoot)
    {
        return arenaRoot.GetComponentsInChildren<Collider>(true)
            .Where(item => item.gameObject.activeInHierarchy && item.name.StartsWith("Obstacle_"))
            .Select(item => item.bounds)
            .ToList();
    }

    private static Transform FindBall(Transform root)
    {
        return root.GetComponentsInChildren<Transform>(true)
            .FirstOrDefault(item => item.name.StartsWith("TargetBall"));
    }

    private static void PrintGroup(string name, List<Result> group)
    {
        float startVisible = group.Count(item => item.VisibleAtStart) * 100f / group.Count;
        Debug.Log($"ACTIVE_SEARCH_GROUP|kind={name}|count={group.Count}|" +
                  $"visibleAtStart={startVisible:F1}%|" +
                  $"distanceToDetection={group.Average(item => item.DistanceUntilDetection):F2}m|" +
                  $"active={group.Average(item => item.ActiveSeconds):F2}s|" +
                  $"stopScan={group.Average(item => item.StopAndScanSeconds):F2}s");
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new InvalidOperationException(message);
        }
    }
}
