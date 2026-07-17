using System;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using Unity.InferenceEngine;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEditor.Build;
using UnityEngine;

public static class ROSBridgeSceneSetup
{
    private const string ScenePath = "Assets/Scenes/SampleScene.unity";

    [MenuItem("GFS-X/Configure Real Robot ROS Bridge")]
    public static void ConfigureRealRobotBridge()
    {
        string[] defines = PlayerSettings.GetScriptingDefineSymbols(NamedBuildTarget.Standalone)
            .Split(new[] { ';' }, StringSplitOptions.RemoveEmptyEntries)
            .Where(item => !string.Equals(item, "ROS2", StringComparison.OrdinalIgnoreCase))
            .ToArray();
        PlayerSettings.SetScriptingDefineSymbols(NamedBuildTarget.Standalone, string.Join(";", defines));

        var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        RobotBrain brain = UnityEngine.Object.FindObjectsByType<RobotBrain>(
                FindObjectsInactive.Include, FindObjectsSortMode.None)
            .OrderBy(item => item.transform.root.GetSiblingIndex())
            .ThenBy(item => item.transform.GetSiblingIndex())
            .FirstOrDefault();
        if (brain == null)
        {
            throw new InvalidOperationException("No GFSX robot with RobotBrain was found.");
        }

        ROSBridge[] oldBridges = UnityEngine.Object.FindObjectsByType<ROSBridge>(
            FindObjectsInactive.Include, FindObjectsSortMode.None);
        foreach (ROSBridge oldBridge in oldBridges.Where(item => item.gameObject != brain.gameObject))
        {
            UnityEngine.Object.DestroyImmediate(oldBridge);
        }

        ROSBridge bridge = brain.GetComponent<ROSBridge>();
        if (bridge == null)
        {
            bridge = brain.gameObject.AddComponent<ROSBridge>();
        }

        DiagnosticLogger logger = brain.GetComponent<DiagnosticLogger>();
        if (logger == null)
        {
            logger = brain.gameObject.AddComponent<DiagnosticLogger>();
        }

        VirtualSensors sensors = brain.GetComponent<VirtualSensors>();
        SimulatedYoloCamera camera = brain.GetComponentInChildren<SimulatedYoloCamera>(true);
        RealVision vision = brain.GetComponent<RealVision>();
        if (vision == null)
        {
            vision = brain.gameObject.AddComponent<RealVision>();
        }
        vision.Configure(camera);
        var bridgeObject = new SerializedObject(bridge);
        bridgeObject.FindProperty("realRobotMode").boolValue = true;
        bridgeObject.FindProperty("sensorTarget").objectReferenceValue = sensors;
        bridgeObject.FindProperty("cameraTarget").objectReferenceValue = camera;
        bridgeObject.FindProperty("realVision").objectReferenceValue = vision;
        bridgeObject.FindProperty("maxLinearSpeed").floatValue = 0.25f;
        bridgeObject.FindProperty("frontSafetyStopDistance").floatValue = 0.50f;
        bridgeObject.FindProperty("cameraYawLimitDegrees").floatValue = 70f;
        bridgeObject.ApplyModifiedPropertiesWithoutUndo();

        var brainObject = new SerializedObject(brain);
        brainObject.FindProperty("rosBridge").objectReferenceValue = bridge;
        brainObject.FindProperty("diagnosticLogger").objectReferenceValue = logger;
        brainObject.ApplyModifiedPropertiesWithoutUndo();

        BehaviorParameters behavior = brain.GetComponent<BehaviorParameters>();
        ModelAsset model = AssetDatabase.LoadAssetAtPath<ModelAsset>("Assets/GFSX_Brain2.onnx") ??
                           AssetDatabase.LoadAssetAtPath<ModelAsset>("Assets/GFSX_Brain.onnx");
        if (behavior == null || model == null)
        {
            throw new InvalidOperationException("Behavior Parameters or GFSX ONNX model was not found.");
        }

        behavior.Model = model;
        behavior.BehaviorType = BehaviorType.InferenceOnly;

        ROSConnection connection = UnityEngine.Object.FindFirstObjectByType<ROSConnection>(
            FindObjectsInactive.Include);
        if (connection == null)
        {
            var connectionObject = new GameObject("ROSConnection");
            connection = connectionObject.AddComponent<ROSConnection>();
        }

        var connectionSettings = new SerializedObject(connection);
        connectionSettings.FindProperty("m_ConnectOnStart").boolValue = true;
        connectionSettings.ApplyModifiedPropertiesWithoutUndo();

        EditorUtility.SetDirty(bridge);
        EditorUtility.SetDirty(vision);
        EditorUtility.SetDirty(logger);
        EditorUtility.SetDirty(brain);
        EditorUtility.SetDirty(behavior);
        EditorUtility.SetDirty(connection);
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        Debug.Log($"Configured ROSBridge on {brain.transform.root.name}/{brain.name}. " +
                  "Set the Raspberry Pi address in Robotics > ROS Settings before Play.");
    }
}
