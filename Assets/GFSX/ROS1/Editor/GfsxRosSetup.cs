using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class GfsxRosSetup
{
    private const string RobotName = "GFSX_Robot";

    [MenuItem("Tools/URFU/Configure ROS1 Bridge")]
    public static void ConfigureBridge()
    {
        GameObject robot = GameObject.Find(RobotName);
        if (robot == null)
        {
            throw new InvalidOperationException($"Не найден объект {RobotName} в открытой сцене.");
        }

        var driveAdapter = GetOrAdd<GfsxDriveAdapter>(robot);
        var gripperAdapter = GetOrAdd<GfsxGripperAdapter>(robot);
        var sensorAdapter = GetOrAdd<GfsxSensorAdapter>(robot);
        var bridge = GetOrAdd<GfsxRosBridge>(robot);

        Component driveController = FindComponentWithMethod(robot, "SetCommand", typeof(float), typeof(float));
        Component gripperController = FindComponentByTypeName(robot, "GripperController");
        Component virtualSensors = FindComponentByTypeName(robot, "VirtualSensors");

        driveAdapter.Configure(driveController);
        gripperAdapter.Configure(gripperController);
        sensorAdapter.Configure(virtualSensors);

        Behaviour[] competingControllers = robot
            .GetComponentsInChildren<Behaviour>(true)
            .Where(component => component != null && component.GetType().Name == "RobotBrain")
            .ToArray();

        bridge.Configure(driveAdapter, gripperAdapter, sensorAdapter, competingControllers);

        ROSConnection connection = ROSConnection.GetOrCreateInstance();
        Undo.RecordObject(connection, "Configure GFS-X ROS1 connection");
        connection.RosIPAddress = "127.0.0.1";
        connection.RosPort = 10000;
        connection.ConnectOnStart = false;

        EditorUtility.SetDirty(robot);
        EditorUtility.SetDirty(driveAdapter);
        EditorUtility.SetDirty(gripperAdapter);
        EditorUtility.SetDirty(sensorAdapter);
        EditorUtility.SetDirty(bridge);
        EditorUtility.SetDirty(connection);
        EditorSceneManager.MarkSceneDirty(robot.scene);
        Selection.activeGameObject = robot;

        ValidateBridge();
        Debug.Log("GFS-X ROS1 bridge настроен. Проверьте компоненты на GFSX_Robot и сохраните сцену.");
    }

    [MenuItem("Tools/URFU/Validate ROS1 Bridge")]
    public static void ValidateBridge()
    {
        var problems = new List<string>();
        GameObject robot = GameObject.Find(RobotName);

        if (robot == null)
        {
            problems.Add($"Нет объекта {RobotName}.");
        }
        else
        {
            var bridge = robot.GetComponent<GfsxRosBridge>();
            var drive = robot.GetComponent<GfsxDriveAdapter>();
            var gripper = robot.GetComponent<GfsxGripperAdapter>();
            var sensors = robot.GetComponent<GfsxSensorAdapter>();

            if (bridge == null) problems.Add("Нет GfsxRosBridge.");
            if (drive == null || drive.TargetController == null) problems.Add("Не назначен контроллер движения.");
            if (gripper == null || gripper.TargetController == null) problems.Add("Не назначен GripperController.");
            if (sensors == null || sensors.TargetSensors == null) problems.Add("Не назначен VirtualSensors.");
        }

        int connectionCount = UnityEngine.Object.FindObjectsByType<ROSConnection>(FindObjectsInactive.Include, FindObjectsSortMode.None).Length;
        if (connectionCount != 1)
        {
            problems.Add($"В сцене должно быть одно ROSConnection, сейчас: {connectionCount}.");
        }

        string defines = PlayerSettings.GetScriptingDefineSymbolsForGroup(EditorUserBuildSettings.selectedBuildTargetGroup);
        if (defines.Split(';').Contains("ROS2"))
        {
            problems.Add("В Robotics → ROS Settings необходимо выбрать Protocol = ROS1.");
        }

        if (problems.Count > 0)
        {
            throw new InvalidOperationException("ROS1 validation failed:\n- " + string.Join("\n- ", problems));
        }

        Debug.Log("GFS-X ROS1 bridge validation passed.");
    }

    private static T GetOrAdd<T>(GameObject target) where T : Component
    {
        T component = target.GetComponent<T>();
        return component != null ? component : Undo.AddComponent<T>(target);
    }

    private static Component FindComponentByTypeName(GameObject robot, string typeName)
    {
        return robot
            .GetComponentsInChildren<Component>(true)
            .FirstOrDefault(component => component != null && component.GetType().Name == typeName);
    }

    private static Component FindComponentWithMethod(GameObject robot, string methodName, params Type[] parameterTypes)
    {
        foreach (Component component in robot.GetComponentsInChildren<Component>(true))
        {
            if (component == null)
            {
                continue;
            }

            MethodInfo method = component.GetType().GetMethod(
                methodName,
                BindingFlags.Instance | BindingFlags.Public,
                null,
                parameterTypes,
                null);

            if (method != null)
            {
                return component;
            }
        }

        return null;
    }
}
