using System;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class GfsxYoloVisionCommands
{
    private const string RobotName = "GFSX_Robot";
    private const string CameraName = "RobotCamera";

    [MenuItem("Tools/GFSX YOLO/Use Simulated Vision")]
    public static void UseSimulatedVision()
    {
        SetVision(simulated: true);
    }

    [MenuItem("Tools/GFSX YOLO/Use Real Vision")]
    public static void UseRealVision()
    {
        SetVision(simulated: false);
    }

    private static void SetVision(bool simulated)
    {
        GameObject robot = GameObject.Find(RobotName);
        if (robot == null)
        {
            throw new InvalidOperationException($"Object {RobotName} was not found in the active scene.");
        }

        RobotBrain brain = robot.GetComponentInChildren<RobotBrain>(true);
        if (brain == null)
        {
            throw new InvalidOperationException("RobotBrain was not found under GFSX_Robot.");
        }

        Transform cameraTransform = FindChildRecursive(robot.transform, CameraName);
        if (cameraTransform == null)
        {
            throw new InvalidOperationException($"{CameraName} was not found under GFSX_Robot.");
        }

        SimulatedYoloCamera simulatedCamera = GetOrAdd<SimulatedYoloCamera>(cameraTransform.gameObject);
        RealYoloCamera realCamera = GetOrAdd<RealYoloCamera>(cameraTransform.gameObject);

        simulatedCamera.enabled = simulated;
        realCamera.enabled = !simulated;
        brain.SetVisionSource(simulated ? simulatedCamera : realCamera);

        EditorUtility.SetDirty(brain);
        EditorUtility.SetDirty(simulatedCamera);
        EditorUtility.SetDirty(realCamera);
        EditorSceneManager.MarkSceneDirty(robot.scene);
        Selection.activeGameObject = robot;

        Debug.Log(simulated
            ? "GFS-X YOLO vision switched to SimulatedYoloCamera."
            : "GFS-X YOLO vision switched to RealYoloCamera.");
    }

    private static T GetOrAdd<T>(GameObject target) where T : Component
    {
        T component = target.GetComponent<T>();
        return component != null ? component : Undo.AddComponent<T>(target);
    }

    private static Transform FindChildRecursive(Transform parent, string childName)
    {
        foreach (Transform child in parent)
        {
            if (child.name == childName)
            {
                return child;
            }

            Transform nested = FindChildRecursive(child, childName);
            if (nested != null)
            {
                return nested;
            }
        }

        return null;
    }
}
