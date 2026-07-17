using System;
using System.IO;
using Unity.MLAgents.Policies;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class TestVersionValidator
{
    [UnityEditor.MenuItem("Tools/URFU/Validate Test Version")]
    public static void Validate()
    {
        const string scenePath = "Assets/Scenes/SampleScene.unity";
        EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);

        RobotBrain[] brains = UnityEngine.Object.FindObjectsByType<RobotBrain>(
            FindObjectsInactive.Include, FindObjectsSortMode.None);
        ArenaObstacleRandomizer[] arenas = UnityEngine.Object.FindObjectsByType<ArenaObstacleRandomizer>(
            FindObjectsInactive.Include, FindObjectsSortMode.None);
        Require(brains.Length == 40, $"Expected 40 RobotBrain components, found {brains.Length}.");
        Require(arenas.Length == 40, $"Expected 40 arenas, found {arenas.Length}.");

        foreach (RobotBrain brain in brains)
        {
            BehaviorParameters behavior = brain.GetComponent<BehaviorParameters>();
            Require(behavior != null, $"BehaviorParameters missing on {brain.name}.");
            Require(behavior.BrainParameters.VectorObservationSize == 15,
                $"{brain.name}: expected 15 observations.");
            Require(behavior.BrainParameters.NumStackedVectorObservations == 1,
                $"{brain.name}: expected one observation stack.");
            Require(behavior.BrainParameters.ActionSpec.NumContinuousActions == 3,
                $"{brain.name}: expected three continuous actions.");
            Require(behavior.BrainParameters.ActionSpec.NumDiscreteActions == 1 &&
                    behavior.BrainParameters.ActionSpec.BranchSizes[0] == 2,
                $"{brain.name}: expected one binary grab action.");
        }

        Require(File.Exists("Assets/Scenes/EvaluationScene.unity"), "EvaluationScene is missing.");
        Require(File.Exists("config.yaml"), "config.yaml is missing.");
        Debug.Log($"TEST_VERSION_VALIDATION PASSED: {brains.Length} robots, {arenas.Length} arenas.");
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new InvalidOperationException("Test version validation failed: " + message);
        }
    }
}
