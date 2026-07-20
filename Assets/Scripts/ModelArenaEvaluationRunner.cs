using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public sealed class ModelArenaEvaluationRunner : MonoBehaviour
{
    private const float ScenarioDuration = 35f;
    private RobotBrain robot;
    private Rigidbody robotBody;
    private Transform ball;
    private Rigidbody ballBody;
    private SimulatedYoloCamera vision;
    private GripperController gripper;
    private Vector3 robotStart;
    private Quaternion robotRotation;
    private Vector3 ballStart;
    private Quaternion ballRotation;
    private GameObject wall;

    private IEnumerator Start()
    {
        if (!HasArgument("--model-eval"))
        {
            yield break;
        }

        Application.runInBackground = true;
        Time.timeScale = 8f;
        SelectSingleRobot();
        if (robot == null || ball == null || vision == null)
        {
            Debug.LogError("MODEL_EVAL_FATAL|required scene objects were not found");
            Application.Quit(2);
            yield break;
        }

        robotStart = robot.transform.position;
        robotRotation = robot.transform.rotation;
        robot.SetEvaluationBoundaryRadius(100f);
        ballStart = ball.position;
        ballRotation = ball.rotation;

        for (int trial = 1; trial <= 3; trial++)
        {
            yield return RunScenario($"visible_{trial}", 0.80f, false);
            yield return RunScenario($"far_{trial}", 3.00f, false);
            yield return RunScenario($"occluded_{trial}", 2.50f, true);
        }

        Debug.Log("MODEL_EVAL_COMPLETE");
        Application.Quit(0);
    }

    private IEnumerator RunScenario(string scenario, float ballDistance, bool withWall)
    {
        robot.EndEpisode();
        yield return new WaitForFixedUpdate();
        ResetScene(ballDistance, withWall);
        // Measure the exact reset pose before the policy or ball physics gets
        // a chance to modify the initial conditions.
        vision.EvaluateTarget();

        bool visibleAtStart = vision.IsVisible;
        bool everVisible = visibleAtStart;
        float firstSeen = visibleAtStart ? 0f : -1f;
        float elapsed = 0f;
        float travel = 0f;
        int successCountAtStart = robot.SuccessfulEpisodeCount;
        int episodeSequenceAtStart = robot.EpisodeSequence;
        Vector3 previous = robot.transform.position;
        var cells = new HashSet<Vector2Int>();

        while (elapsed < ScenarioDuration && robot.SuccessfulEpisodeCount == successCountAtStart)
        {
            yield return new WaitForFixedUpdate();
            elapsed += Time.fixedDeltaTime;
            Vector3 current = robot.transform.position;
            travel += Vector3.ProjectOnPlane(current - previous, Vector3.up).magnitude;
            previous = current;
            Vector3 local = current - robotStart;
            cells.Add(new Vector2Int(Mathf.RoundToInt(local.x / 0.8f), Mathf.RoundToInt(local.z / 0.8f)));

            vision.EvaluateTarget();
            if (vision.IsVisible && !everVisible)
            {
                everVisible = true;
                firstSeen = elapsed;
            }
        }

        float finalDistance = Vector3.Distance(robot.transform.position, ball.position);
        bool grabbed = robot.SuccessfulEpisodeCount > successCountAtStart;
        int unexpectedResets = robot.EpisodeSequence - episodeSequenceAtStart;
        Debug.Log($"MODEL_EVAL_RESULT|scenario={scenario}|visibleAtStart={visibleAtStart}|" +
                  $"everVisible={everVisible}|firstSeen={firstSeen:F2}|grabbed={grabbed}|" +
                  $"elapsed={elapsed:F2}|travel={travel:F2}|cells={cells.Count}|resets={unexpectedResets}|" +
                  $"finalDistance={finalDistance:F2}");
    }

    private void ResetScene(float ballDistance, bool withWall)
    {
        if (wall != null)
        {
            // Destroy is deferred until the end of the frame. Disable the old
            // barrier immediately so it cannot occlude the following scenario.
            wall.SetActive(false);
            Destroy(wall);
            wall = null;
        }
        gripper.Release();
        robot.transform.SetPositionAndRotation(robotStart, robotRotation);
        robot.ResetEvaluationActuators();
        robotBody.linearVelocity = Vector3.zero;
        robotBody.angularVelocity = Vector3.zero;

        Vector3 direction = robot.transform.right.normalized;
        Vector3 forwardOrigin = gripper.HoldPoint != null
            ? gripper.HoldPoint.position
            : robotStart;
        ball.SetParent(null, true);
        Vector3 ballPosition = forwardOrigin + direction * ballDistance;
        ballPosition.y = ballStart.y;
        ball.SetPositionAndRotation(ballPosition, ballRotation);
        if (ballBody != null)
        {
            ballBody.isKinematic = false;
            ballBody.linearVelocity = Vector3.zero;
            ballBody.angularVelocity = Vector3.zero;
        }

        if (withWall)
        {
            wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
            wall.name = "Obstacle_ModelEvaluationBarrier";
            wall.transform.position = Vector3.Lerp(forwardOrigin, ball.position, 0.5f);
            wall.transform.position = new Vector3(wall.transform.position.x, robotStart.y + 0.65f, wall.transform.position.z);
            wall.transform.rotation = Quaternion.LookRotation(direction, Vector3.up);
            wall.transform.localScale = new Vector3(3.0f, 1.3f, 0.55f);
        }

        Physics.SyncTransforms();
    }

    private void SelectSingleRobot()
    {
        RobotBrain[] robots = FindObjectsByType<RobotBrain>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        if (robots.Length == 0) return;
        robot = robots[0];
        for (int index = 1; index < robots.Length; index++) robots[index].gameObject.SetActive(false);

        ArenaObstacleRandomizer robotArena = robot.GetComponentInParent<ArenaObstacleRandomizer>();
        foreach (ArenaObstacleRandomizer randomizer in FindObjectsByType<ArenaObstacleRandomizer>(FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            if (randomizer != robotArena)
            {
                randomizer.gameObject.SetActive(false);
                continue;
            }

            randomizer.enabled = false;
            foreach (Transform item in randomizer.GetComponentsInChildren<Transform>(true))
                if (item.name.StartsWith("Obstacle_")) item.gameObject.SetActive(false);
        }

        robot.gameObject.SetActive(true);
        robotBody = robot.GetComponent<Rigidbody>();
        vision = robot.GetComponentInChildren<SimulatedYoloCamera>(true);
        gripper = robot.GetComponent<GripperController>();
        if (robotArena != null)
        {
            foreach (Transform candidate in robotArena.GetComponentsInChildren<Transform>(true))
            {
                if (candidate.CompareTag("TargetBall"))
                {
                    ball = candidate;
                    break;
                }
            }
        }

        if (ball == null)
        {
            GameObject ballObject = GameObject.FindGameObjectWithTag("TargetBall");
            ball = ballObject != null ? ballObject.transform : null;
        }
        ballBody = ball != null ? ball.GetComponent<Rigidbody>() : null;
    }

    private static bool HasArgument(string expected)
    {
        foreach (string argument in System.Environment.GetCommandLineArgs())
            if (argument == expected) return true;
        return false;
    }
}
