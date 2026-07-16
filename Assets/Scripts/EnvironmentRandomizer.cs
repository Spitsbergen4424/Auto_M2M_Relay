using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;

// Intended home is the shared parent of one training area (GFSX_Robot, TargetBall,
// TrainingArena all nested under it, as in Assets/Prefab/Scene.prefab). RobotBrain only
// ever calls Randomize() from OnEpisodeBegin - it does not know or care how positions are
// chosen, so this class can be re-tuned (or replaced) without touching agent/reward code.
// Reference resolution tries a local child/sibling lookup first: once a build instantiates
// many training areas side by side, that is the only way to reach *this* instance's own
// robot/ball instead of an arbitrary one. It falls back to a scene-wide GameObject.Find
// only when nothing local was found, which is safe while a single training area is loaded
// (e.g. testing straight in SampleScene before it is wrapped into that shared parent).
public sealed class EnvironmentRandomizer : MonoBehaviour
{
    [Header("Scene references (auto-resolved by name if left empty)")]
    [SerializeField] private Transform floor;
    [SerializeField] private Rigidbody robotBody;
    [SerializeField] private Rigidbody ballBody;
    [SerializeField] private GameObject obstaclePrefab;

    [Header("Spawn area")]
    [SerializeField] private float wallMargin = 0.6f;
    [SerializeField] private int placementAttempts = 20;

    [Header("Obstacles (Obstacle_type_1)")]
    [SerializeField] private int minObstacles = 3;
    [SerializeField] private int maxObstacles = 8;
    [SerializeField] private Vector2 obstacleScaleRange = new Vector2(0.5f, 2f);
    [SerializeField] private float obstacleClearance = 0.55f;

    [Header("Robot / ball placement")]
    [SerializeField] private float robotClearance = 0.9f;
    [SerializeField] private float ballClearance = 0.35f;
    [SerializeField] private float ballMinDistanceFromRobot = 3.0f;
    [SerializeField] private float ballMaxDistanceFromRobot = 6.0f;

    private readonly List<Transform> obstaclePool = new List<Transform>();
    private readonly List<Vector3> occupiedPoints = new List<Vector3>();
    private readonly List<float> occupiedRadii = new List<float>();

    private Vector3 baseObstacleScale;
    private float obstacleSpawnY;
    private float robotSpawnY;
    private float ballSpawnY;
    private float baseBallMass;
    private Vector3 baseBallScale;
    private Vector2 areaMin;
    private Vector2 areaMax;
    private bool ready;

    private void Awake()
    {
        // Explicit null checks (not ??=) - Unity's overloaded null check on destroyed/
        // missing Object references isn't honoured by ??, so this is the safe form for
        // UnityEngine.Object fields.
        if (floor == null)
        {
            floor = ResolveFloor();
        }
        if (robotBody == null)
        {
            robotBody = ResolveSibling("GFSX_Robot")?.GetComponent<Rigidbody>();
        }
        if (ballBody == null)
        {
            ballBody = ResolveSibling("TargetBall")?.GetComponent<Rigidbody>();
        }

        robotSpawnY = robotBody != null ? robotBody.position.y : transform.position.y;
        ballSpawnY = ballBody != null ? ballBody.position.y : transform.position.y;
        if (ballBody != null)
        {
            baseBallMass = ballBody.mass;
            baseBallScale = ballBody.transform.localScale;
        }

        BuildObstaclePool();
        // Obstacles are optional (obstaclePrefab may not be assigned yet) - robot/ball
        // placement should still work as long as the floor is there to bound it.
        ready = floor != null;
    }

    private void BuildObstaclePool()
    {
        if (obstaclePrefab == null)
        {
            return;
        }

        baseObstacleScale = obstaclePrefab.transform.localScale;
        obstacleSpawnY = obstaclePrefab.transform.position.y;

        Transform pool = transform.Find("ObstaclePool");
        if (pool == null)
        {
            pool = new GameObject("ObstaclePool").transform;
            pool.SetParent(transform, false);
        }

        // Pre-instantiate the largest possible batch once and reuse it every episode
        // (enable/reposition instead of Instantiate/Destroy) - with dozens of training
        // areas resetting in parallel, per-episode allocation/GC would be the bottleneck.
        for (int i = 0; i < maxObstacles; i++)
        {
            GameObject instance = Instantiate(obstaclePrefab, pool);
            instance.SetActive(false);
            obstaclePool.Add(instance.transform);
        }
    }

    // Raw floor footprint (not shrunk by wallMargin - that margin only keeps spawns away
    // from the walls, it is not the boundary of the playable surface). RobotBrain uses this
    // to detect a genuine "left the arena" instead of an arbitrary distance-from-start
    // radius, so it stays correct no matter how far apart domain randomization spreads the
    // robot and ball.
    public bool TryGetFloorBounds(out Vector2 min, out Vector2 max)
    {
        Renderer floorRenderer = floor != null ? floor.GetComponent<Renderer>() : null;
        if (floorRenderer == null)
        {
            min = default;
            max = default;
            return false;
        }

        Bounds bounds = floorRenderer.bounds;
        min = new Vector2(bounds.min.x, bounds.min.z);
        max = new Vector2(bounds.max.x, bounds.max.z);
        return true;
    }

    public void Randomize()
    {
        if (!ready)
        {
            return;
        }

        // Read live overrides from config.yaml's environment_parameters (via mlagents-learn)
        // every episode, falling back to the Inspector value when nothing was sent - that
        // covers both "no trainer attached" (manual/heuristic play) and "key not in yaml".
        float ballMinDistance = GetParam("ball_min_distance", ballMinDistanceFromRobot);
        float ballMaxDistance = GetParam("ball_max_distance", ballMaxDistanceFromRobot);
        float scaleMin = GetParam("obstacle_scale_min", obstacleScaleRange.x);
        float scaleMax = GetParam("obstacle_scale_max", obstacleScaleRange.y);
        int obstacleMin = Mathf.RoundToInt(GetParam("obstacle_min_count", minObstacles));
        // The pool itself is sized from maxObstacles once at Awake (build time), so a
        // config value asking for more than that ceiling is clamped, not honoured -
        // raising the real ceiling still needs a bigger maxObstacles and a rebuild.
        int obstacleMax = Mathf.Clamp(Mathf.RoundToInt(GetParam("obstacle_max_count", maxObstacles)),
            0, obstaclePool.Count);

        ComputeArea();
        occupiedPoints.Clear();
        occupiedRadii.Clear();

        Vector3 robotPoint = SampleUniform();
        if (robotBody != null)
        {
            robotBody.position = new Vector3(robotPoint.x, robotSpawnY, robotPoint.z);
            robotBody.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
            robotBody.linearVelocity = Vector3.zero;
            robotBody.angularVelocity = Vector3.zero;
        }
        Occupy(robotPoint, robotClearance);

        if (ballBody != null)
        {
            if (!TryFindPointNear(robotPoint, ballMinDistance, ballMaxDistance, ballClearance,
                    out Vector3 ballPoint))
            {
                TryFindUniformPoint(ballClearance, out ballPoint);
            }

            ballBody.position = new Vector3(ballPoint.x, ballSpawnY, ballPoint.z);
            ballBody.rotation = Quaternion.identity;
            ballBody.linearVelocity = Vector3.zero;
            ballBody.angularVelocity = Vector3.zero;
            Occupy(ballPoint, ballClearance);

            // Domain randomization (training only): a real ball's mass/size varies
            // (different balls, wear, humidity), and the gripper/vision code should not be
            // able to overfit to one exact value.
            if (IsTraining())
            {
                // Floor raised from 0.3x to 0.5x: an ultra-light ball colliding with a much
                // heavier robot/obstacle is what was causing PhysX to eject it at extreme
                // speed on an overlapping spawn, spiking the distance-to-ball reward.
                ballBody.mass = Random.Range(baseBallMass * 0.5f, baseBallMass * 1.7f);
                ballBody.transform.localScale = baseBallScale * Random.Range(0.8f, 1.2f);
            }
            else
            {
                ballBody.mass = baseBallMass;
                ballBody.transform.localScale = baseBallScale;
            }
        }

        int count = Random.Range(obstacleMin, obstacleMax + 1);
        for (int i = 0; i < obstaclePool.Count; i++)
        {
            Transform instance = obstaclePool[i];
            if (i < count && TryFindUniformPoint(obstacleClearance, out Vector3 point))
            {
                instance.gameObject.SetActive(true);
                instance.position = new Vector3(point.x, obstacleSpawnY, point.z);
                instance.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
                instance.localScale = new Vector3(
                    baseObstacleScale.x * Random.Range(scaleMin, scaleMax),
                    baseObstacleScale.y * Random.Range(scaleMin, scaleMax),
                    baseObstacleScale.z * Random.Range(scaleMin, scaleMax));
                Occupy(point, obstacleClearance);
            }
            else
            {
                // Not enough clear space left this episode - leave it parked and hidden
                // rather than forcing an overlap.
                instance.gameObject.SetActive(false);
            }
        }

        Physics.SyncTransforms();
    }

    private static float GetParam(string key, float defaultValue)
    {
        return Academy.IsInitialized
            ? Academy.Instance.EnvironmentParameters.GetWithDefault(key, defaultValue)
            : defaultValue;
    }

    // True only when mlagents-learn is actually attached - not during manual/heuristic play
    // or a baked-model inference build, so those keep clean, noise-free physics.
    private static bool IsTraining()
    {
        return Academy.IsInitialized && Academy.Instance.IsCommunicatorOn;
    }

    private Transform ResolveSibling(string objectName)
    {
        Transform local = transform.Find(objectName) ?? transform.parent?.Find(objectName);
        if (local != null)
        {
            return local;
        }

        GameObject found = GameObject.Find(objectName);
        return found != null ? found.transform : null;
    }

    private Transform ResolveFloor()
    {
        Transform local = transform.Find("TrainingArena/Floor") ?? transform.Find("Floor")
            ?? transform.parent?.Find("TrainingArena/Floor");
        if (local != null)
        {
            return local;
        }

        GameObject arena = GameObject.Find("TrainingArena");
        return arena != null ? arena.transform.Find("Floor") : null;
    }

    private void ComputeArea()
    {
        Renderer floorRenderer = floor.GetComponent<Renderer>();
        Bounds bounds = floorRenderer != null
            ? floorRenderer.bounds
            : new Bounds(floor.position, new Vector3(10f, 0f, 10f));
        areaMin = new Vector2(bounds.min.x + wallMargin, bounds.min.z + wallMargin);
        areaMax = new Vector2(bounds.max.x - wallMargin, bounds.max.z - wallMargin);
    }

    private Vector3 SampleUniform()
    {
        return new Vector3(Random.Range(areaMin.x, areaMax.x), 0f, Random.Range(areaMin.y, areaMax.y));
    }

    private bool TryFindUniformPoint(float clearance, out Vector3 point)
    {
        for (int attempt = 0; attempt < placementAttempts; attempt++)
        {
            Vector3 candidate = SampleUniform();
            if (IsClear(candidate, clearance))
            {
                point = candidate;
                return true;
            }
        }

        point = SampleUniform();
        return false;
    }

    private bool TryFindPointNear(Vector3 anchor, float minDistance, float maxDistance, float clearance,
        out Vector3 point)
    {
        for (int attempt = 0; attempt < placementAttempts; attempt++)
        {
            float angle = Random.Range(0f, Mathf.PI * 2f);
            float distance = Random.Range(minDistance, maxDistance);
            Vector3 candidate = anchor + new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle)) * distance;
            candidate.x = Mathf.Clamp(candidate.x, areaMin.x, areaMax.x);
            candidate.z = Mathf.Clamp(candidate.z, areaMin.y, areaMax.y);
            if (IsClear(candidate, clearance))
            {
                point = candidate;
                return true;
            }
        }

        point = anchor;
        return false;
    }

    private bool IsClear(Vector3 point, float radius)
    {
        for (int i = 0; i < occupiedPoints.Count; i++)
        {
            float required = radius + occupiedRadii[i];
            if ((point - occupiedPoints[i]).sqrMagnitude < required * required)
            {
                return false;
            }
        }

        return true;
    }

    private void Occupy(Vector3 point, float radius)
    {
        occupiedPoints.Add(point);
        occupiedRadii.Add(radius);
    }
}
