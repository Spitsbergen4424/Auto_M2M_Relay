using System;
using System.IO;
using System.Linq;
using Unity.InferenceEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;


public static class GfsxRealRobotSetup
{
    private const string ScenePath = "Assets/Scenes/RealRobotScene.unity";
    private const string RobotPrefabPath = "Assets/prefab/123.prefab";
    private const string RobotName = "GFSX_Robot";
    private const string BrainModelPath = "Assets/GFSX_Brain-799994.onnx";

    [MenuItem("Tools/URFU/Configure Real Robot Scene")]
    public static void Configure()
    {
        Scene scene = OpenOrCreateScene();
        ClearScene(scene);

        GameObject environment = new GameObject("RealRobotEnvironment");
        GameObject robotPrefab =
            AssetDatabase.LoadAssetAtPath<GameObject>(RobotPrefabPath);

        if (robotPrefab == null)
        {
            throw new InvalidOperationException(
                $"Robot prefab was not found at {RobotPrefabPath}.");
        }

        GameObject robot =
            (GameObject)PrefabUtility.InstantiatePrefab(robotPrefab);

        robot.name = RobotName;
        robot.transform.SetParent(environment.transform, false);
        robot.transform.localPosition = Vector3.zero;
        robot.transform.localRotation = Quaternion.Euler(0f, -90f, 0f);

        RobotBrain brain = robot.GetComponentInChildren<RobotBrain>(true);
        if (brain == null)
        {
            throw new InvalidOperationException(
                $"RobotBrain was not found in {robot.name} or its children.");
        }

        GameObject robotController = brain.gameObject;

        RealRobotSensors realSensors =
            GetOrAdd<RealRobotSensors>(robotController);

        GfsxRealRobotBridge bridge =
            GetOrAdd<GfsxRealRobotBridge>(robotController);

        DiagnosticLogger diagnosticLogger =
            GetOrAdd<DiagnosticLogger>(robotController);

        SimulatedYoloCamera simulatedCamera =
            robot.GetComponentInChildren<SimulatedYoloCamera>(true);

        TrackController trackController =
            GetRequiredComponent<TrackController>(robotController);

        GripperController gripper =
            GetRequiredComponent<GripperController>(robotController);

        Rigidbody robotBody =
            GetRequiredComponent<Rigidbody>(robotController);

        robotBody.useGravity = false;
        robotBody.isKinematic = true;


        Transform robotCameraTransform =
            FindChildRecursive(robot.transform, "RobotCamera");

        if (robotCameraTransform == null)
        {
            throw new InvalidOperationException(
                "RobotCamera child is missing on the robot prefab.");
        }

        RealYoloCamera realCamera =
            GetOrAdd<RealYoloCamera>(robotCameraTransform.gameObject);

        realSensors.Configure(2.0f);
        realSensors.ConfigureFreshnessTimeout(0.5f);
        realCamera.Configure(5005, 0.35f);

        bridge.ConfigureRealMode(
            "192.168.2.154",
            10000,
            true,
            false,
            0.25f,
            0.9f,
            10f,
            false,
            0.30f,
            false);

        brain.SetSensorSource(realSensors);
        brain.SetPoseSource(bridge);
        brain.SetVisionSource(realCamera);
        brain.SetExternalActuationEnabled(true);
        ConfigureDiagnosticLogger(diagnosticLogger);

        bridge.enabled = true;

        if (simulatedCamera != null)
        {
            simulatedCamera.enabled = false;
        }

        DisableIfFound<RobotCameraViewport>(robot);
        DisableIfFound<VirtualSensors>(robot);
        DisableIfFound<TrackController>(robot);

        realCamera.enabled = true;

        BehaviorParameters behavior =
            GetRequiredComponent<BehaviorParameters>(robotController);

        ModelAsset model =
            AssetDatabase.LoadAssetAtPath<ModelAsset>(
                BrainModelPath);

        if (model == null)
        {
            throw new InvalidOperationException(
                "Assets/GFSX_Brain.onnx could not be imported as a ModelAsset.");
        }

        behavior.BehaviorName = "GFSX_Brain";
        behavior.Model = model;
        behavior.BehaviorType = BehaviorType.InferenceOnly;
        behavior.BrainParameters.VectorObservationSize = 15;
        behavior.BrainParameters.NumStackedVectorObservations = 1;
        behavior.BrainParameters.ActionSpec =
            new ActionSpec(3, new[] { 2 });

        brain.MaxStep = 0;

        DecisionRequester requester =
            GetRequiredComponent<DecisionRequester>(robotController);

        requester.DecisionPeriod = 5;
        requester.DecisionStep = 0;
        requester.TakeActionsBetweenDecisions = true;

        ROSConnection connection = EnsureRosConnection();
        connection.RosIPAddress = "192.168.2.154";
        connection.RosPort = 10000;
        connection.ConnectOnStart = false;

        // Реальный GFS-X не использует ROS TF в текущем контракте.
        // Отключаем стандартную подписку на /tf, чтобы Unity не требовала
        // отсутствующий сгенерированный тип tf2_msgs/TFMessage.
        connection.listenForTFMessages = false;
        connection.TFTopics = Array.Empty<string>();

        SetupCameraRig(robot);
        EnsureOverviewCamera(robot.transform);
        RemoveTrainingOnlyChildren(robot.transform);
        DisableLegacyRosBridge(robot);

        EditorUtility.SetDirty(environment);
        EditorUtility.SetDirty(robot);
        EditorUtility.SetDirty(robotController);
        EditorUtility.SetDirty(realSensors);
        EditorUtility.SetDirty(bridge);
        EditorUtility.SetDirty(diagnosticLogger);
        EditorUtility.SetDirty(brain);
        EditorUtility.SetDirty(realCamera);
        EditorUtility.SetDirty(trackController);
        EditorUtility.SetDirty(gripper);
        EditorUtility.SetDirty(behavior);
        EditorUtility.SetDirty(requester);
        EditorUtility.SetDirty(connection);
        EditorUtility.SetDirty(robotBody);


        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene, ScenePath);
        AssetDatabase.SaveAssets();

        Validate();

        Selection.activeGameObject = robot;
        Debug.Log("Real robot scene configured and validated.");
    }

    [MenuItem("Tools/URFU/Validate Real Robot Scene")]
    public static void Validate()
    {
        if (!File.Exists(ScenePath))
        {
            throw new InvalidOperationException(
                "RealRobotScene is missing. Run Configure Real Robot Scene first.");
        }

        Scene scene =
            EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);

        RobotBrain[] brains =
            UnityEngine.Object.FindObjectsByType<RobotBrain>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);

        RealYoloCamera[] realCameras =
            UnityEngine.Object.FindObjectsByType<RealYoloCamera>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);

        GfsxRealRobotBridge[] realBridges =
            UnityEngine.Object.FindObjectsByType<GfsxRealRobotBridge>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);

        GfsxRosBridge[] simBridges =
            UnityEngine.Object.FindObjectsByType<GfsxRosBridge>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);

        RealRobotSensors[] sensors =
            UnityEngine.Object.FindObjectsByType<RealRobotSensors>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);

        Require(
            brains.Length == 1,
            $"Expected one RobotBrain, found {brains.Length}.");

        Require(
            realCameras.Length == 1,
            $"Expected one RealYoloCamera, found {realCameras.Length}.");

        Require(
            realBridges.Length == 1,
            $"Expected one GfsxRealRobotBridge, found {realBridges.Length}.");

        Require(
            sensors.Length == 1,
            $"Expected one RealRobotSensors, found {sensors.Length}.");

        RobotBrain brain = brains[0];
        RealYoloCamera realCamera = realCameras[0];
        GfsxRealRobotBridge bridge = realBridges[0];
        RealRobotSensors realSensors = sensors[0];

        Require(brain.enabled, "RobotBrain must be enabled.");
        Require(realCamera.enabled, "RealYoloCamera must be enabled.");
        Require(bridge.enabled, "GfsxRealRobotBridge must be enabled.");

        Require(
            brain.SensorSource == realSensors,
            "RobotBrain must use RealRobotSensors as its sensor source.");

        Require(
            brain.PoseSource == bridge,
            "RobotBrain must use GfsxRealRobotBridge as its pose source.");

        DiagnosticLogger[] loggers =
            UnityEngine.Object.FindObjectsByType<DiagnosticLogger>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);

        Require(
            loggers.Length == 1,
            $"Expected one DiagnosticLogger, found {loggers.Length}.");

        DiagnosticLogger diagnosticLogger = loggers[0];
        Require(
            brain.GetComponent<DiagnosticLogger>() == diagnosticLogger,
            "DiagnosticLogger must be attached to the RobotBrain GameObject.");

        SerializedObject loggerObject = new SerializedObject(diagnosticLogger);
        Require(
            loggerObject.FindProperty("enableLogging").boolValue,
            "DiagnosticLogger must have logging enabled.");
        Require(
            loggerObject.FindProperty("logEveryN").intValue == 1,
            "DiagnosticLogger logEveryN must be 1.");
        Require(
            loggerObject.FindProperty("maxRows").intValue == 2000,
            "DiagnosticLogger maxRows must be 2000.");
        Require(
            loggerObject.FindProperty("flushEveryNRows").intValue == 10,
            "DiagnosticLogger flushEveryNRows must be 10.");
        Require(
            loggerObject.FindProperty("fileName").stringValue == "diagnostic_log.csv",
            "DiagnosticLogger fileName must be diagnostic_log.csv.");

        Require(
            realCamera.ListenPort == 5005,
            $"RealYoloCamera must listen on 5005, found {realCamera.ListenPort}.");

        Require(
            bridge.RosIpAddress == "192.168.2.154",
            $"ROS IP must be 192.168.2.154, found {bridge.RosIpAddress}.");

        Require(
            bridge.RosPort == 10000,
            $"ROS port must be 10000, found {bridge.RosPort}.");

        Require(
            bridge.DryRun,
            "Dry run must be enabled by default.");

        Require(
            !bridge.EnableMotorCommands,
            "Motor commands must be disabled by default.");

        Require(
            !bridge.EnableGripperCommands,
            "Gripper commands must be disabled by default.");

        Require(
            Mathf.Approximately(
                bridge.SafetyStopDistanceMeters,
                0.30f),
            $"Safety stop distance must be 0.30 m, found {bridge.SafetyStopDistanceMeters}.");

        Require(
            bridge.StopAfterCapture,
            "Physical bridge must stop and latch after gripper capture.");

        Require(
            bridge.MaxLinearSpeedMetersPerSecond >= 0.175f,
            "Linear command cap is below the robot's effective 35 PWM motor threshold.");

        Require(
            bridge.MaxAngularSpeedRadiansPerSecond >= 0.70f,
            "Angular command cap is below the robot's effective differential-drive threshold.");

        Require(
            Mathf.Approximately(
                realSensors.UltrasonicMaxDistanceMeters,
                2.0f),
            "Ultrasonic normalization must use the 2.0 m training contract.");

        TrackController trackController =
            brain.GetComponent<TrackController>();

        Require(
            trackController == null || !trackController.enabled,
            "TrackController must be disabled in RealRobotScene.");

        SimulatedYoloCamera simulatedCamera =
            brain.GetComponentInChildren<SimulatedYoloCamera>(true);

        Require(
            simulatedCamera == null || !simulatedCamera.enabled,
            "SimulatedYoloCamera must be disabled in RealRobotScene.");

        BehaviorParameters behavior =
            brain.GetComponent<BehaviorParameters>();

        Require(
            behavior != null,
            "BehaviorParameters is missing on RobotBrain.");

        Require(
            behavior.BehaviorName == "GFSX_Brain",
            "BehaviorName must remain GFSX_Brain.");

        Require(
            behavior.Model != null,
            "Assets/GFSX_Brain.onnx must be assigned for physical inference.");

        Require(
            behavior.BehaviorType == BehaviorType.InferenceOnly,
            "RealRobotScene must use InferenceOnly so it cannot silently fall back to Heuristic.");

        Require(
            brain.MaxStep == 0,
            "MaxStep must be zero for a continuous physical-robot mission.");

        Require(
            behavior.BrainParameters.VectorObservationSize == 15,
            "RealRobotScene must keep the 15-observation PPO interface.");

        Require(
            behavior.BrainParameters.NumStackedVectorObservations == 1,
            "RealRobotScene must keep one observation stack.");

        Require(
            behavior.BrainParameters.ActionSpec.NumContinuousActions == 3,
            "RealRobotScene must keep three continuous actions.");

        Require(
            behavior.BrainParameters.ActionSpec.NumDiscreteActions == 1 &&
            behavior.BrainParameters.ActionSpec.BranchSizes[0] == 2,
            "RealRobotScene must keep one binary discrete branch.");

        Require(
            behavior.Model ==
            AssetDatabase.LoadAssetAtPath<ModelAsset>(BrainModelPath),
            $"RealRobotScene must use {BrainModelPath}.");


        int bridgeCount =
            UnityEngine.Object.FindObjectsByType<GfsxRealRobotBridge>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None).Length;

        int simBridgeCount =
            UnityEngine.Object.FindObjectsByType<GfsxRosBridge>(
                    FindObjectsInactive.Include,
                    FindObjectsSortMode.None)
                .Count(item => item.enabled);

        Require(
            bridgeCount == 1,
            $"Expected one physical bridge, found {bridgeCount}.");

        Require(
            simBridgeCount == 0,
            "Simulation GfsxRosBridge must be disabled or absent.");

        int rosConnectionCount =
            UnityEngine.Object.FindObjectsByType<ROSConnection>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None).Length;


        ROSConnection[] rosConnections =
            UnityEngine.Object.FindObjectsByType<ROSConnection>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);

        Require(
            rosConnections.Length == 1,
            $"Expected one ROSConnection, found {rosConnections.Length}.");

        Require(
            !rosConnections[0].listenForTFMessages,
            "ROS TF listening must be disabled for the current GFS-X contract.");

        Require(
            rosConnections[0].TFTopics == null ||
            rosConnections[0].TFTopics.Length == 0,
            "ROS TF topic list must be empty.");

        Debug.Log("REAL_ROBOT_SCENE validation passed.");
    }

    private static Scene OpenOrCreateScene()
    {
        if (File.Exists(ScenePath))
        {
            return EditorSceneManager.OpenScene(
                ScenePath,
                OpenSceneMode.Single);
        }

        Scene scene =
            EditorSceneManager.NewScene(
                NewSceneSetup.EmptyScene,
                NewSceneMode.Single);

        Directory.CreateDirectory(
            Path.GetDirectoryName(ScenePath) ?? "Assets/Scenes");

        return scene;
    }

    private static void ClearScene(Scene scene)
    {
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            UnityEngine.Object.DestroyImmediate(root);
        }
    }

    private static void ConfigureDiagnosticLogger(DiagnosticLogger logger)
    {
        logger.enabled = true;
        SerializedObject serializedLogger = new SerializedObject(logger);
        serializedLogger.FindProperty("enableLogging").boolValue = true;
        serializedLogger.FindProperty("logEveryN").intValue = 1;
        serializedLogger.FindProperty("maxRows").intValue = 2000;
        serializedLogger.FindProperty("flushEveryNRows").intValue = 10;
        serializedLogger.FindProperty("fileName").stringValue = "diagnostic_log.csv";
        serializedLogger.ApplyModifiedPropertiesWithoutUndo();
    }

    private static ROSConnection EnsureRosConnection()
    {
        ROSConnection connection =
            UnityEngine.Object.FindFirstObjectByType<ROSConnection>();

        if (connection != null)
        {
            return connection;
        }

        GameObject go = new GameObject("ROSConnection");
        connection = Undo.AddComponent<ROSConnection>(go);

        return connection;
    }

    private static void SetupCameraRig(GameObject robot)
    {
        Transform cameraPivot =
            FindChildRecursive(robot.transform, "CameraPivot");

        Transform cameraTransform =
            FindChildRecursive(robot.transform, "RobotCamera");

        if (cameraTransform == null)
        {
            return;
        }

        Camera camera = cameraTransform.GetComponent<Camera>();
        if (camera != null)
        {
            camera.enabled = false;
        }

        if (cameraPivot != null)
        {
            cameraPivot.gameObject.SetActive(true);
        }
    }

    private static void EnsureOverviewCamera(Transform robot)
    {
        Camera overview =
            UnityEngine.Object.FindObjectsByType<Camera>(
                    FindObjectsInactive.Include,
                    FindObjectsSortMode.None)
                .FirstOrDefault(item =>
                    item != null &&
                    item.enabled &&
                    !item.transform.IsChildOf(robot));

        if (overview == null)
        {
            GameObject cameraObject = new GameObject("Main Camera");
            overview = cameraObject.AddComponent<Camera>();
        }

        overview.enabled = true;
        overview.tag = "MainCamera";

        // Обзорная камера создаётся вне Prefab instance.
        Transform environment = robot.parent;
        overview.transform.SetParent(environment, false);

        overview.transform.position =
            robot.TransformPoint(
                new Vector3(-3.2f, 1.8f, -1.8f));

        overview.transform.LookAt(
            robot.TransformPoint(
                new Vector3(0f, 0.8f, 1.0f)));
    }


    private static void RemoveTrainingOnlyChildren(Transform root)
    {
        GameObject[] objectsToRemove =
            root.GetComponentsInChildren<Transform>(true)
                .Where(child =>
                    child != null &&
                    child != root &&
                    (
                        child.name.StartsWith(
                            "TrainingArena",
                            StringComparison.OrdinalIgnoreCase) ||
                        child.name.StartsWith(
                            "TargetBall",
                            StringComparison.OrdinalIgnoreCase)
                    ))
                .Select(child => child.gameObject)
                .Distinct()
                .ToArray();

        // Список сначала формируется полностью, и только потом объекты удаляются.
        // Это предотвращает обращение к Transform, уничтоженным вместе с родителем.
        foreach (GameObject item in objectsToRemove)
        {
            if (item != null)
            {
                UnityEngine.Object.DestroyImmediate(item);
            }
        }
    }

    private static void DisableLegacyRosBridge(GameObject robot)
    {
        foreach (GfsxRosBridge bridge in
                 robot.GetComponentsInChildren<GfsxRosBridge>(true))
        {
            bridge.enabled = false;
        }
    }

    private static void DisableIfFound<T>(GameObject root)
        where T : Behaviour
    {
        T component = root.GetComponentInChildren<T>(true);

        if (component != null)
        {
            component.enabled = false;
        }
    }

    private static T GetOrAdd<T>(GameObject target)
        where T : Component
    {
        T component = target.GetComponent<T>();

        return component != null
            ? component
            : Undo.AddComponent<T>(target);
    }

    private static T GetRequiredComponent<T>(GameObject target)
        where T : Component
    {
        T component = target.GetComponent<T>();

        if (component == null)
        {
            throw new InvalidOperationException(
                $"{typeof(T).Name} is missing on {target.name}.");
        }

        return component;
    }

    private static Transform FindChildRecursive(
        Transform root,
        string childName)
    {
        foreach (Transform child in
                 root.GetComponentsInChildren<Transform>(true))
        {
            if (child.name.Equals(
                    childName,
                    StringComparison.OrdinalIgnoreCase))
            {
                return child;
            }
        }

        return null;
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new InvalidOperationException(
                "Real robot scene validation failed: " + message);
        }
    }
}
