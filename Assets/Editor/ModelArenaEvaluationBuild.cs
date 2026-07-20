using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using Unity.InferenceEngine;
using Unity.MLAgents.Policies;
using UnityEngine;

public static class ModelArenaEvaluationBuild
{
    private const string SourceScene = "Assets/Scenes/SampleScene.unity";
    private const string EvaluationScene = "Assets/Scenes/ModelArenaEvaluation.unity";

    public static void Build()
    {
        EditorSceneManager.OpenScene(SourceScene);
        ModelAsset model = AssetDatabase.LoadAssetAtPath<ModelAsset>("Assets/GFSX_Brain_RewardV2.onnx");
        if (model == null)
            model = AssetDatabase.LoadAssetAtPath<ModelAsset>("Assets/GFSX_Brain2.onnx");
        if (model == null) throw new System.Exception("No evaluation ONNX model was found.");

        foreach (BehaviorParameters behavior in Object.FindObjectsByType<BehaviorParameters>(FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            behavior.BehaviorType = BehaviorType.InferenceOnly;
            behavior.Model = model;
        }

        GameObject runner = new GameObject("ModelArenaEvaluationRunner");
        runner.AddComponent<ModelArenaEvaluationRunner>();
        EditorSceneManager.SaveScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene(), EvaluationScene, true);

        string outputDirectory = Path.GetFullPath("ModelEvaluationBuild");
        Directory.CreateDirectory(outputDirectory);
        BuildPlayerOptions options = new BuildPlayerOptions
        {
            scenes = new[] { EvaluationScene },
            locationPathName = Path.Combine(outputDirectory, "ModelEvaluation.exe"),
            target = BuildTarget.StandaloneWindows64,
            options = BuildOptions.None
        };
        BuildReport report = BuildPipeline.BuildPlayer(options);
        if (report.summary.result != BuildResult.Succeeded)
            throw new System.Exception($"Evaluation build failed: {report.summary.result}");
        Debug.Log($"MODEL_EVAL_BUILD_READY|{outputDirectory}");
    }
}
