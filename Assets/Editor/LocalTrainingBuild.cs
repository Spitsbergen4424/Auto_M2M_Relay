using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using Unity.MLAgents.Policies;
using UnityEngine;

public static class LocalTrainingBuild
{
    private const string SourceScene = "Assets/Scenes/SampleScene.unity";
    private const string TrainingScene = "Assets/Scenes/LocalTrainingScene.unity";

    public static void Build()
    {
        EditorSceneManager.OpenScene(SourceScene);

        foreach (BehaviorParameters behavior in Object.FindObjectsByType<BehaviorParameters>(
                     FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            behavior.BehaviorType = BehaviorType.Default;
            behavior.Model = null;
        }

        EditorSceneManager.SaveScene(
            UnityEngine.SceneManagement.SceneManager.GetActiveScene(), TrainingScene, true);

        string outputDirectory = Path.GetFullPath("LocalTrainingBuild");
        Directory.CreateDirectory(outputDirectory);
        BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
        {
            scenes = new[] { TrainingScene },
            locationPathName = Path.Combine(outputDirectory, "GFSX_Training.exe"),
            target = BuildTarget.StandaloneWindows64,
            options = BuildOptions.None
        });

        if (report.summary.result != BuildResult.Succeeded)
            throw new System.Exception($"Local training build failed: {report.summary.result}");

        Debug.Log($"LOCAL_TRAINING_BUILD_READY|{outputDirectory}");
    }
}
