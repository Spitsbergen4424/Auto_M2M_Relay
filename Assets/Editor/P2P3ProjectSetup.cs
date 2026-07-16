using System;
using System.IO;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class P2P3ProjectSetup
{
    private const string ScenePath = "Assets/Scenes/SampleScene.unity";
    private const string BuildPath = "Build/GFSX_Simulator.exe";
    private const float ScaleMultiplier = 7f;
    private const float ModelScale = 0.7f;

    [MenuItem("Tools/URFU/Configure P2 + P3")]
    public static void Configure()
    {
        EnsureTag("TargetBall");
        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);

        GameObject robot = FindRobot(scene);
        robot.name = "GFSX_Robot";
        robot.transform.SetParent(null);
        // The supplied FBX is authored in decimeter-like units and faces local +X.
        // Scale it to the real robot footprint and align its nose with Unity's +Z convention.
        robot.transform.SetPositionAndRotation(Vector3.zero, Quaternion.Euler(0f, -90f, 0f));
        robot.transform.localScale = Vector3.one * ModelScale;

        RemoveOldTestAgents(robot);
        RemoveLegacyRootCamera(robot);
        DisableAutomaticRosConnection();
        ConfigureManipulatorPose(robot);

        PrefabUtility.RecordPrefabInstancePropertyModifications(robot.transform);
        Bounds visualBounds = CalculateBounds(robot);
        Bounds localVisualBounds = CalculateLocalBounds(robot, null);
        Bounds bodyBounds = CalculateLocalBounds(robot, RendererUsesBodyMaterial);
        if (bodyBounds.size.sqrMagnitude < 0.01f)
        {
            bodyBounds = localVisualBounds;
        }

        Vector3 localCenter = bodyBounds.center;
        Vector3 size = bodyBounds.size;

        Rigidbody body = GetOrAdd<Rigidbody>(robot);
        body.mass = 2.5f;
        body.linearDamping = 8f;
        body.angularDamping = 8f;
        body.useGravity = true;
        body.isKinematic = false;
        body.interpolation = RigidbodyInterpolation.Interpolate;
        body.collisionDetectionMode = CollisionDetectionMode.Continuous;
        body.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

        BoxCollider box = GetOrAdd<BoxCollider>(robot);
        box.isTrigger = false;
        box.center = new Vector3(localCenter.x, bodyBounds.min.y + size.y * 0.38f, localCenter.z);
        box.size = new Vector3(size.x * 0.88f, size.y * 0.62f, size.z * 0.86f);

        CapsuleCollider bumper = GetOrAdd<CapsuleCollider>(robot);
        bumper.isTrigger = false;
        bumper.direction = 2;
        bumper.radius = Mathf.Max(0.12f, size.y * 0.16f);
        bumper.height = Mathf.Max(bumper.radius * 2f, size.z * 0.82f);
        bumper.center = new Vector3(bodyBounds.max.x - bumper.radius, box.center.y, localCenter.z);
        ConfigurePartColliders(robot);
        ConfigureOpenClawCollider(robot);
        ConfigureFingerColliders(robot);

        // Model-space calibration: nose +X, left +Z, right -Z.
        float front = bodyBounds.max.x + size.x * 0.025f;
        float sensorY = bodyBounds.min.y + size.y * 0.48f;
        Transform centerPoint = CreatePoint(robot.transform, "CenterPoint",
            new Vector3(front, sensorY, localCenter.z), new Vector3(0f, 90f, 0f));
        Transform leftPoint = CreatePoint(robot.transform, "LeftIRPoint",
            new Vector3(bodyBounds.center.x, sensorY, bodyBounds.max.z), Vector3.zero);
        Transform rightPoint = CreatePoint(robot.transform, "RightIRPoint",
            new Vector3(bodyBounds.center.x, sensorY, bodyBounds.min.z), new Vector3(0f, 180f, 0f));

        Renderer openClaw = FindOpenClawRenderer(robot);
        Bounds clawBounds = openClaw != null
            ? CalculateRendererBoundsInRoot(openClaw, robot.transform)
            : new Bounds(new Vector3(front, sensorY, localCenter.z), new Vector3(1.2f, 1.2f, 1.2f));
        // At the requested x7 scale the visual claw base sits far behind the actual front
        // tips. Put the logical capture gate at the front opening and at ball-centre height.
        Vector3 gripCenter = new Vector3(
            localVisualBounds.max.x - size.x * 0.08f,
            bodyBounds.min.y + 0.40f,
            clawBounds.center.z);
        Transform gripperPoint = CreatePoint(robot.transform, "GripperIRPoint", gripCenter,
            new Vector3(0f, 90f, 0f));
        Transform holdPoint = CreatePoint(robot.transform, "HoldPoint", gripCenter,
            new Vector3(0f, 90f, 0f));

        TrackController tracks = GetOrAdd<TrackController>(robot);
        tracks.ConfigureScale(ScaleMultiplier);
        VirtualSensors sensors = GetOrAdd<VirtualSensors>(robot);
        sensors.ConfigureScale(ScaleMultiplier);
        sensors.Configure(centerPoint, leftPoint, rightPoint, gripperPoint);
        GripperController gripper = GetOrAdd<GripperController>(robot);
        gripper.Configure(holdPoint, sensors, 0.70f);

        GameObject ball = ConfigureBall(scene, visualBounds.min.y);
        // The camera is mounted on the model's Plane.016 part. This makes the
        // bucket view follow the arm instead of floating with the robot body.
        Transform cameraMount = FindChildRecursive(robot.transform, "Plane.016");
        Transform cameraPivot = FindChildRecursive(robot.transform, "CameraPivot");
        if (cameraPivot == null)
        {
            cameraPivot = CreatePoint(robot.transform, "CameraPivot",
                new Vector3(clawBounds.max.x + size.x * 0.20f, localVisualBounds.max.y + size.y * 0.35f,
                    clawBounds.center.z - size.z * 0.22f), Vector3.zero);
        }
        if (cameraMount != null)
        {
            cameraPivot.SetParent(cameraMount, true);
            cameraPivot.SetPositionAndRotation(cameraMount.position, cameraMount.rotation);
        }
        GameObject cameraObject = GetOrCreateChild(cameraPivot, "RobotCamera");
        Vector3 bucketViewTarget = robot.transform.TransformPoint(new Vector3(
            clawBounds.max.x - size.x * 0.18f, clawBounds.center.y, clawBounds.center.z));
        Quaternion bucketViewRotation = Quaternion.LookRotation(
            bucketViewTarget - cameraPivot.position, robot.transform.up);
        cameraObject.transform.SetPositionAndRotation(cameraPivot.position, bucketViewRotation);
        Camera robotCamera = GetOrAdd<Camera>(cameraObject);
        robotCamera.enabled = true;
        robotCamera.rect = new Rect(0.80f, 0.66f, 0.18f, 0.32f);
        robotCamera.depth = 10f;
        robotCamera.fieldOfView = 40f;
        robotCamera.nearClipPlane = 0.05f;
        robotCamera.farClipPlane = 50f;
        SimulatedYoloCamera yolo = GetOrAdd<SimulatedYoloCamera>(cameraObject);
        yolo.Configure(robotCamera, ball.transform);
        yolo.ConfigureScale(ScaleMultiplier);
        RobotCameraViewport viewport = GetOrAdd<RobotCameraViewport>(cameraObject);
        viewport.Configure(0.18f, 0.02f);

        RobotBrain brain = GetOrAdd<RobotBrain>(robot);
        // Longer than the original 5000: with domain randomization spreading the robot and
        // ball up to ball_max_distance apart and obstacles forcing detours, reaching the
        // ball legitimately takes more decision steps than the original short-range layout.
        brain.MaxStep = 8000;
        brain.Configure(tracks, sensors, gripper, yolo, cameraPivot, ball.transform);

        BehaviorParameters behavior = GetOrAdd<BehaviorParameters>(robot);
        behavior.BehaviorName = "GFSX_Brain";
        // Default selects Heuristic automatically without a trainer/model and Remote Policy
        // when mlagents-learn connects, so manual WASD and P3 training both remain available.
        behavior.BehaviorType = BehaviorType.Default;
        behavior.BrainParameters.VectorObservationSize = 15;
        behavior.BrainParameters.NumStackedVectorObservations = 4;
        behavior.BrainParameters.ActionSpec = new ActionSpec(3, new[] { 3 });

        DecisionRequester requester = GetOrAdd<DecisionRequester>(robot);
        requester.DecisionPeriod = 5;
        requester.DecisionStep = 0;
        requester.TakeActionsBetweenDecisions = true;

        ConfigureArena(scene, visualBounds.min.y);
        ConfigureMainCamera(robot.transform, robotCamera);
        EnsureBuildScene();

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        AssetDatabase.SaveAssets();
        Validate();
        Debug.Log("P2/P3 configuration completed and validated.");
    }

    [MenuItem("Tools/URFU/Validate P2 + P3")]
    public static void Validate()
    {
        GameObject robot = GameObject.Find("GFSX_Robot");
        Require(robot != null, "GFSX_Robot is missing.");
        Require(robot.transform.position == Vector3.zero, "Robot Transform Position must be (0,0,0).");
        Require(robot.GetComponent<Rigidbody>() != null, "Rigidbody is missing.");
        Require(robot.GetComponent<BoxCollider>() != null && robot.GetComponent<CapsuleCollider>() != null,
            "Body or bumper collider is missing.");
        foreach (Renderer renderer in robot.GetComponentsInChildren<Renderer>(true))
        {
            Require(renderer.GetComponent<Collider>() != null ||
                    renderer.transform.Find("__OpenClawColliders") != null ||
                    renderer.transform.Find("__FingerColliders") != null,
                $"Detailed collider is missing on rendered part: {renderer.name}.");
        }
        Require(robot.GetComponent<TrackController>() != null, "TrackController is missing.");
        Require(robot.GetComponent<VirtualSensors>() != null, "VirtualSensors is missing.");
        Require(robot.GetComponent<GripperController>() != null, "GripperController is missing.");
        Require(robot.GetComponent<RobotBrain>() != null, "RobotBrain is missing.");
        Require(robot.transform.Find("CenterPoint") != null && robot.transform.Find("LeftIRPoint") != null &&
                robot.transform.Find("RightIRPoint") != null && robot.transform.Find("GripperIRPoint") != null &&
                robot.transform.Find("HoldPoint") != null, "One or more P2 anchor points are missing.");
        Require(Vector3.Dot(robot.transform.Find("CenterPoint").forward, robot.transform.right) > 0.99f,
            "Center sensor does not face the FBX nose (+X).");
        Camera robotCamera = robot.transform.Find("CameraPivot/RobotCamera")?.GetComponent<Camera>();
        Require(robotCamera != null && robotCamera.enabled, "Robot camera must be visible for Q/E control.");
        Require(Mathf.Approximately(robotCamera.fieldOfView, 40f), "Robot camera FOV must be 40 degrees.");
        Require(robotCamera.rect.width < 0.3f && robotCamera.rect.height < 0.5f,
            "Robot camera must be a small corner viewport.");
        Require(!robotCamera.CompareTag("MainCamera"), "Robot camera must be the secondary camera.");
        Require(Camera.main != null && Camera.main != robotCamera && Camera.main.enabled,
            "Overview Main Camera must be enabled.");
        Require(robot.transform.localScale == Vector3.one * ModelScale, "Robot model scale must be 0.7.");

        BehaviorParameters behavior = robot.GetComponent<BehaviorParameters>();
        Require(behavior != null && behavior.BehaviorName == "GFSX_Brain", "Behavior Parameters are missing.");
        Require(behavior.BehaviorType == BehaviorType.Default,
            "Behavior Type must be Default so mlagents-learn can connect.");
        Require(behavior.BrainParameters.VectorObservationSize == 15 &&
                behavior.BrainParameters.NumStackedVectorObservations == 4, "Observation layout must be 15 x 4.");
        Require(behavior.BrainParameters.ActionSpec.NumContinuousActions == 3 &&
                behavior.BrainParameters.ActionSpec.NumDiscreteActions == 1 &&
                behavior.BrainParameters.ActionSpec.BranchSizes[0] == 3, "Action layout must be 3 continuous + [3].");
        Require(robot.GetComponent<DecisionRequester>()?.DecisionPeriod == 5, "Decision Period must be 5.");
        Require(GameObject.FindWithTag("TargetBall") != null, "TargetBall is missing.");
        Require(File.Exists("config.yaml"), "config.yaml is missing.");
        ValidateTrainingConfig();
    }

    [MenuItem("Tools/URFU/Build GFS-X Simulator")]
    public static void BuildSimulator()
    {
        Configure();
        Directory.CreateDirectory("Build");
        BuildPlayerOptions options = new BuildPlayerOptions
        {
            scenes = new[] { ScenePath },
            locationPathName = BuildPath,
            target = BuildTarget.StandaloneWindows64,
            options = BuildOptions.None
        };
        BuildReport report = BuildPipeline.BuildPlayer(options);
        if (report.summary.result != BuildResult.Succeeded)
        {
            throw new InvalidOperationException($"Build failed: {report.summary.result}, errors: {report.summary.totalErrors}");
        }

        Debug.Log($"GFS-X simulator built: {BuildPath} ({report.summary.totalSize} bytes).");
    }

    public static void ConfigureAndBuild()
    {
        BuildSimulator();
    }

    public static void ConfigureAndCaptureCamera()
    {
        Configure();
        CaptureRobotCameraPreview();
    }

    public static void TestGripperCalibration()
    {
        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        GameObject robot = GameObject.Find("GFSX_Robot");
        GameObject ball = GameObject.FindWithTag("TargetBall");
        GripperController gripper = robot?.GetComponent<GripperController>();
        Require(gripper != null && gripper.HoldPoint != null && ball != null,
            "Gripper test prerequisites are missing.");

        Transform originalParent = ball.transform.parent;
        Vector3 originalPosition = ball.transform.position;
        Quaternion originalRotation = ball.transform.rotation;
        Rigidbody ballBody = ball.GetComponent<Rigidbody>();
        bool originalKinematic = ballBody != null && ballBody.isKinematic;

        ball.transform.SetParent(null, true);
        ball.transform.position = gripper.HoldPoint.position + gripper.HoldPoint.forward * 0.15f;
        Physics.SyncTransforms();
        gripper.TryGrab();
        Require(gripper.HasBall && ball.transform.parent == gripper.HoldPoint,
            "Space/grab calibration test failed.");
        Require(ballBody == null || ballBody.isKinematic, "Grab must make the ball kinematic.");

        GameObject released = gripper.Release();
        Require(released == ball && !gripper.HasBall && ball.transform.parent == null,
            "R/release calibration test failed.");
        Require(ballBody == null || !ballBody.isKinematic, "Release must restore ball physics.");
        foreach (Collider collider in ball.GetComponentsInChildren<Collider>(true))
        {
            Require(collider.enabled, "Release must restore ball colliders.");
        }

        ball.transform.SetParent(originalParent, true);
        ball.transform.SetPositionAndRotation(originalPosition, originalRotation);
        if (ballBody != null) ballBody.isKinematic = originalKinematic;
        Physics.SyncTransforms();
        Debug.Log("GRIPPER_CALIBRATION_TEST passed: grab, parenting, release and physics restore.");
    }

    public static void ReportRobotColliders()
    {
        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        GameObject robot = FindRobot(scene);
        foreach (Renderer renderer in robot.GetComponentsInChildren<Renderer>(true))
        {
            Collider collider = renderer.GetComponent<Collider>();
            string materials = string.Join(",", Array.ConvertAll(renderer.sharedMaterials,
                material => material == null ? "null" : material.name));
            Vector3 rootLocalCenter = robot.transform.InverseTransformPoint(renderer.bounds.center);
            Vector3 axisX = robot.transform.InverseTransformDirection(renderer.transform.right).normalized;
            Vector3 axisY = robot.transform.InverseTransformDirection(renderer.transform.up).normalized;
            Vector3 axisZ = robot.transform.InverseTransformDirection(renderer.transform.forward).normalized;
            Debug.Log($"COLLIDER_REPORT path={GetPath(renderer.transform, robot.transform)} " +
                      $"renderer={renderer.GetType().Name} rootLocalCenter={rootLocalCenter} worldSize={renderer.bounds.size} " +
                      $"localPosition={renderer.transform.localPosition} localEuler={renderer.transform.localEulerAngles} " +
                      $"axes=({axisX}|{axisY}|{axisZ}) localScale={renderer.transform.localScale} materials={materials} " +
                      $"collider={(collider == null ? "NONE" : collider.GetType().Name)}");
        }
    }

    public static void CaptureRobotCameraPreview()
    {
        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        GameObject robot = GameObject.Find("GFSX_Robot");
        Camera camera = robot?.transform.Find("CameraPivot/RobotCamera")?.GetComponent<Camera>();
        Require(camera != null, "Robot camera is missing for preview capture.");

        const int width = 720;
        const int height = 420;
        RenderTexture texture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
        RenderTexture previousActive = RenderTexture.active;
        RenderTexture previousTarget = camera.targetTexture;
        camera.targetTexture = texture;
        camera.Render();
        RenderTexture.active = texture;
        Texture2D image = new Texture2D(width, height, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        image.Apply();
        Directory.CreateDirectory("Logs");
        File.WriteAllBytes("Logs/RobotCameraPreview.png", image.EncodeToPNG());
        camera.targetTexture = previousTarget;
        RenderTexture.active = previousActive;
        UnityEngine.Object.DestroyImmediate(image);
        texture.Release();
        UnityEngine.Object.DestroyImmediate(texture);
        Debug.Log("Robot camera preview saved to Logs/RobotCameraPreview.png");
    }

    public static void CaptureOverviewPreview()
    {
        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        Camera camera = Camera.main;
        Require(camera != null, "Main camera is missing for overview capture.");
        Vector3 originalPosition = camera.transform.position;
        Quaternion originalRotation = camera.transform.rotation;
        camera.transform.position = new Vector3(-3.2f, 1.8f, -1.8f);
        camera.transform.LookAt(new Vector3(0f, 0.65f, 1.25f));

        const int width = 770;
        const int height = 454;
        RenderTexture texture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
        RenderTexture previousActive = RenderTexture.active;
        RenderTexture previousTarget = camera.targetTexture;
        camera.targetTexture = texture;
        camera.Render();
        RenderTexture.active = texture;
        Texture2D image = new Texture2D(width, height, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        image.Apply();
        Directory.CreateDirectory("Logs");
        File.WriteAllBytes("Logs/RobotOverviewPreview.png", image.EncodeToPNG());
        camera.targetTexture = previousTarget;
        camera.transform.SetPositionAndRotation(originalPosition, originalRotation);
        RenderTexture.active = previousActive;
        UnityEngine.Object.DestroyImmediate(image);
        texture.Release();
        UnityEngine.Object.DestroyImmediate(texture);
        Debug.Log("Robot overview preview saved to Logs/RobotOverviewPreview.png");
    }

    private static GameObject FindRobot(Scene scene)
    {
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            if (root.name.StartsWith("Xiao-r GFS-X_1", StringComparison.OrdinalIgnoreCase) || root.name == "GFSX_Robot")
            {
                return root;
            }
        }

        throw new InvalidOperationException("Imported Xiao-r GFS-X_1 model was not found in SampleScene.");
    }

    private static void RemoveOldTestAgents(GameObject robot)
    {
        TestAgent[] oldAgents = UnityEngine.Object.FindObjectsByType<TestAgent>(FindObjectsInactive.Include);
        foreach (TestAgent old in oldAgents)
        {
            if (old.gameObject != robot)
            {
                UnityEngine.Object.DestroyImmediate(old.gameObject);
            }
            else
            {
                UnityEngine.Object.DestroyImmediate(old);
            }
        }

        foreach (Agent agent in robot.GetComponents<Agent>())
        {
            if (agent is not RobotBrain)
            {
                UnityEngine.Object.DestroyImmediate(agent);
            }
        }
    }

    private static void DisableAutomaticRosConnection()
    {
        foreach (ROSConnection connection in UnityEngine.Object.FindObjectsByType<ROSConnection>(FindObjectsInactive.Include))
        {
            connection.ConnectOnStart = false;
        }
    }

    private static void RemoveLegacyRootCamera(GameObject robot)
    {
        foreach (Component component in robot.GetComponents<Component>())
        {
            if (component == null) continue;
            string typeName = component.GetType().Name;
            if (component is Camera || typeName == "PhysicsRaycaster" || typeName == "UniversalAdditionalCameraData")
            {
                UnityEngine.Object.DestroyImmediate(component);
            }
        }
    }

    private static GameObject ConfigureBall(Scene scene, float floorY)
    {
        GameObject ball = null;
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            if (root.CompareTag("TargetBall") || root.name == "Sphere" || root.name == "TargetBall")
            {
                ball = root;
                break;
            }
        }

        ball ??= GameObject.CreatePrimitive(PrimitiveType.Sphere);
        ball.name = "TargetBall";
        ball.tag = "TargetBall";
        ball.transform.SetParent(null);
        // Preserve the robot/ball proportions after the requested sevenfold scene scale-up.
        ball.transform.SetPositionAndRotation(new Vector3(0f, floorY + 0.28f, 3.5f), Quaternion.identity);
        ball.transform.localScale = Vector3.one * 0.56f;
        Rigidbody body = GetOrAdd<Rigidbody>(ball);
        body.mass = 0.1f;
        body.linearDamping = 0.05f;
        body.angularDamping = 0.05f;
        body.interpolation = RigidbodyInterpolation.Interpolate;
        body.collisionDetectionMode = CollisionDetectionMode.Continuous;
        return ball;
    }

    private static void ConfigureArena(Scene scene, float floorY)
    {
        GameObject arena = GameObject.Find("TrainingArena") ?? new GameObject("TrainingArena");
        arena.transform.SetParent(null);
        arena.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
        CreateArenaCube(arena.transform, "Floor", new Vector3(0f, floorY - 0.05f, 0f), new Vector3(10f, 0.1f, 10f));
        CreateArenaCube(arena.transform, "Wall_North", new Vector3(0f, floorY + 0.5f, 5f), new Vector3(10f, 1f, 0.15f));
        CreateArenaCube(arena.transform, "Wall_South", new Vector3(0f, floorY + 0.5f, -5f), new Vector3(10f, 1f, 0.15f));
        CreateArenaCube(arena.transform, "Wall_East", new Vector3(5f, floorY + 0.5f, 0f), new Vector3(0.15f, 1f, 10f));
        CreateArenaCube(arena.transform, "Wall_West", new Vector3(-5f, floorY + 0.5f, 0f), new Vector3(0.15f, 1f, 10f));

        GameObject legacyFloor = GameObject.Find("Circle.001");
        if (legacyFloor != null)
        {
            legacyFloor.SetActive(false);
        }
    }

    private static void CreateArenaCube(Transform parent, string name, Vector3 position, Vector3 scale)
    {
        GameObject item = GetOrCreateChild(parent, name);
        if (item.GetComponent<MeshFilter>() == null)
        {
            GameObject primitive = GameObject.CreatePrimitive(PrimitiveType.Cube);
            primitive.name = name;
            primitive.transform.SetParent(parent);
            UnityEngine.Object.DestroyImmediate(item);
            item = primitive;
        }

        item.transform.SetPositionAndRotation(position, Quaternion.identity);
        item.transform.localScale = scale;
        GameObjectUtility.SetStaticEditorFlags(item, StaticEditorFlags.BatchingStatic | StaticEditorFlags.OccluderStatic);
    }

    private static void ConfigureMainCamera(Transform robot, Camera robotCamera)
    {
        Camera overview = null;
        foreach (Camera camera in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include,
                     FindObjectsSortMode.None))
        {
            if (camera == robotCamera) continue;
            if (overview == null || camera.gameObject.name == "Main Camera")
            {
                overview = camera;
            }
        }

        if (overview == null)
        {
            GameObject cameraObject = new GameObject("Main Camera");
            overview = cameraObject.AddComponent<Camera>();
        }

        foreach (Camera camera in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include,
                     FindObjectsSortMode.None))
        {
            if (camera != overview && camera != robotCamera) camera.enabled = false;
            if (camera != overview && camera.CompareTag("MainCamera")) camera.tag = "Untagged";
        }

        overview.gameObject.SetActive(true);
        overview.enabled = true;
        overview.tag = "MainCamera";
        overview.rect = new Rect(0f, 0f, 1f, 1f);
        overview.depth = 0f;
        // Follow the robot while staying clearly behind it (negative local Z).
        overview.transform.SetParent(robot, true);
        overview.transform.position = robot.TransformPoint(new Vector3(0f, 4.8f, -6.5f));
        overview.transform.LookAt(robot.TransformPoint(new Vector3(0f, 0.8f, 1.0f)));

        robotCamera.enabled = true;
        robotCamera.tag = "Untagged";
        robotCamera.depth = 10f;
    }

    private static Transform FindChildRecursive(Transform root, string childName)
    {
        foreach (Transform child in root.GetComponentsInChildren<Transform>(true))
        {
            if (child != root && child.name.Equals(childName, StringComparison.OrdinalIgnoreCase))
            {
                return child;
            }
        }

        return null;
    }

    private static Bounds CalculateBounds(GameObject target)
    {
        Renderer[] renderers = target.GetComponentsInChildren<Renderer>(true);
        if (renderers.Length == 0)
        {
            return new Bounds(target.transform.position + Vector3.up * 0.25f, new Vector3(0.75f, 0.5f, 1.15f));
        }

        Bounds bounds = renderers[0].bounds;
        for (int i = 1; i < renderers.Length; i++) bounds.Encapsulate(renderers[i].bounds);
        return bounds;
    }

    private static Bounds CalculateLocalBounds(GameObject target, Func<Renderer, bool> predicate)
    {
        bool initialized = false;
        Bounds combined = default;
        foreach (Renderer renderer in target.GetComponentsInChildren<Renderer>(true))
        {
            if (predicate != null && !predicate(renderer)) continue;
            Bounds item = CalculateRendererBoundsInRoot(renderer, target.transform);
            if (!initialized)
            {
                combined = item;
                initialized = true;
            }
            else
            {
                combined.Encapsulate(item);
            }
        }

        return initialized ? combined : new Bounds(Vector3.zero, Vector3.zero);
    }

    private static Bounds CalculateRendererBoundsInRoot(Renderer renderer, Transform root)
    {
        Bounds local = GetRendererLocalBounds(renderer);
        Vector3 min = local.min;
        Vector3 max = local.max;
        bool initialized = false;
        Bounds result = default;
        for (int x = 0; x < 2; x++)
        for (int y = 0; y < 2; y++)
        for (int z = 0; z < 2; z++)
        {
            Vector3 corner = new Vector3(x == 0 ? min.x : max.x, y == 0 ? min.y : max.y,
                z == 0 ? min.z : max.z);
            Vector3 inRoot = root.InverseTransformPoint(renderer.transform.TransformPoint(corner));
            if (!initialized)
            {
                result = new Bounds(inRoot, Vector3.zero);
                initialized = true;
            }
            else
            {
                result.Encapsulate(inRoot);
            }
        }

        return result;
    }

    private static bool RendererUsesBodyMaterial(Renderer renderer)
    {
        foreach (Material material in renderer.sharedMaterials)
        {
            if (material != null && material.name.StartsWith("Body_", StringComparison.OrdinalIgnoreCase))
            {
                return true;
            }
        }
        return false;
    }

    private static void ConfigurePartColliders(GameObject robot)
    {
        foreach (Renderer renderer in robot.GetComponentsInChildren<Renderer>(true))
        {
            GameObject part = renderer.gameObject;
            if (part == robot)
            {
                continue;
            }

            Collider existing = part.GetComponent<Collider>();
            if (existing != null)
            {
                existing.isTrigger = true;
                continue;
            }

            Bounds bounds = GetRendererLocalBounds(renderer);
            string partName = part.name.ToLowerInvariant();
            if (ContainsAny(partName, "wheel", "roller", "tire", "cylinder"))
            {
                CapsuleCollider capsule = part.AddComponent<CapsuleCollider>();
                capsule.center = bounds.center;
                capsule.direction = LargestAxis(bounds.size);
                float axisSize = capsule.direction == 0 ? bounds.size.x : capsule.direction == 1 ? bounds.size.y : bounds.size.z;
                float sideA = capsule.direction == 0 ? bounds.size.y : bounds.size.x;
                float sideB = capsule.direction == 2 ? bounds.size.y : bounds.size.z;
                capsule.radius = Mathf.Max(0.001f, Mathf.Min(sideA, sideB) * 0.5f);
                capsule.height = Mathf.Max(axisSize, capsule.radius * 2f);
                capsule.isTrigger = true;
            }
            else
            {
                // Convex boxes are stable on a dynamic compound Rigidbody and fit the long,
                // thin claw fingers and track/body panels better than convex MeshColliders.
                BoxCollider detail = part.AddComponent<BoxCollider>();
                detail.center = bounds.center;
                detail.size = bounds.size;
                detail.isTrigger = true;
            }
        }
    }

    private static void ConfigureManipulatorPose(GameObject robot)
    {
        Transform firstJoint = robot.transform.Find("Circle.001");
        Transform middleJoint = robot.transform.Find("Circle.005");
        Transform wrist = robot.transform.Find("Cube.003");
        Transform fingerParent = robot.transform.Find("Cube.003/Circle.002");
        Transform upperFingers = robot.transform.Find("Cube.003/Circle.002/Circle.003");
        Transform lowerFingers = robot.transform.Find("Cube.003/Circle.002/Circle.004");

        Quaternion sourceRotation = Quaternion.Euler(33.022f, 90f, 90f);
        if (firstJoint != null)
        {
            firstJoint.localPosition = new Vector3(1.735662f, 0.7312005f, 0.3777308f);
            firstJoint.localRotation = sourceRotation;
        }
        if (middleJoint != null)
        {
            middleJoint.localPosition = new Vector3(2.537331f, 0.82f, 0.370222f);
            middleJoint.localRotation = sourceRotation;
        }
        if (wrist != null)
        {
            wrist.localPosition = new Vector3(3.08f, 0.80f, 0.35f);
            wrist.localRotation = Quaternion.AngleAxis(-35f, Vector3.forward) * sourceRotation;
        }
        if (fingerParent != null)
        {
            fingerParent.localPosition = Vector3.zero;
            fingerParent.localRotation = Quaternion.identity;
        }

        OpenFingerHalf(robot.transform, upperFingers, -38f);
        OpenFingerHalf(robot.transform, lowerFingers, 38f);
    }

    private static void OpenFingerHalf(Transform robot, Transform fingers, float angle)
    {
        if (fingers == null) return;
        fingers.localRotation = Quaternion.identity;
        fingers.rotation = Quaternion.AngleAxis(angle, robot.up) * fingers.rotation;
    }

    private static void ConfigureFingerColliders(GameObject robot)
    {
        string[] paths =
        {
            "Cube.003/Circle.002/Circle.003",
            "Cube.003/Circle.002/Circle.004"
        };

        foreach (string path in paths)
        {
            Transform fingers = robot.transform.Find(path);
            Renderer renderer = fingers != null ? fingers.GetComponent<Renderer>() : null;
            if (renderer == null) continue;

            foreach (Collider collider in fingers.GetComponents<Collider>())
            {
                UnityEngine.Object.DestroyImmediate(collider);
            }
            Transform old = fingers.Find("__FingerColliders");
            if (old != null) UnityEngine.Object.DestroyImmediate(old.gameObject);

            Bounds bounds = GetRendererLocalBounds(renderer);
            GameObject container = new GameObject("__FingerColliders");
            container.transform.SetParent(fingers, false);

            float baseLength = bounds.size.x * 0.38f;
            GameObject baseObject = new GameObject("BucketBase");
            baseObject.transform.SetParent(container.transform, false);
            BoxCollider baseCollider = baseObject.AddComponent<BoxCollider>();
            baseCollider.center = new Vector3(bounds.min.x + baseLength * 0.5f,
                bounds.center.y, bounds.center.z);
            baseCollider.size = new Vector3(baseLength, bounds.size.y * 0.90f,
                bounds.size.z * 0.78f);
            baseCollider.isTrigger = false;

            const int fingerCount = 4;
            float spacing = bounds.size.y / fingerCount;
            float fingerLength = bounds.size.x - baseLength;
            for (int i = 0; i < fingerCount; i++)
            {
                GameObject item = new GameObject($"Finger_{i + 1}");
                item.transform.SetParent(container.transform, false);
                BoxCollider box = item.AddComponent<BoxCollider>();
                box.center = new Vector3(bounds.min.x + baseLength + fingerLength * 0.5f,
                    bounds.min.y + spacing * (i + 0.5f),
                    bounds.center.z + bounds.size.z * 0.04f);
                box.size = new Vector3(fingerLength * 0.96f, spacing * 0.52f,
                    bounds.size.z * 0.22f);
                box.isTrigger = false;
            }
        }
    }

    private static void ConfigureOpenClawCollider(GameObject robot)
    {
        Renderer renderer = FindOpenClawRenderer(robot);
        if (renderer == null)
        {
            return;
        }

        foreach (Collider oldCollider in renderer.GetComponents<Collider>())
        {
            UnityEngine.Object.DestroyImmediate(oldCollider);
        }

        Transform oldContainer = renderer.transform.Find("__OpenClawColliders");
        if (oldContainer != null)
        {
            UnityEngine.Object.DestroyImmediate(oldContainer.gameObject);
        }

        Bounds bounds = GetRendererLocalBounds(renderer);
        int thinAxis = SmallestAxis(bounds.size);
        int longAxis = LargestAxis(bounds.size);
        int gapAxis = 3 - thinAxis - longAxis;
        float jawThickness = GetAxis(bounds.size, gapAxis) * 0.18f;
        float bridgeThickness = GetAxis(bounds.size, longAxis) * 0.18f;

        GameObject container = new GameObject("__OpenClawColliders");
        container.transform.SetParent(renderer.transform, false);

        Vector3 jawSize = bounds.size;
        SetAxis(ref jawSize, gapAxis, jawThickness);
        SetAxis(ref jawSize, thinAxis, GetAxis(jawSize, thinAxis) * 0.85f);
        float sideOffset = (GetAxis(bounds.size, gapAxis) - jawThickness) * 0.5f;
        CreateClawBox(container.transform, "Jaw_Left", bounds.center, jawSize, gapAxis, -sideOffset);
        CreateClawBox(container.transform, "Jaw_Right", bounds.center, jawSize, gapAxis, sideOffset);

        Vector3 bridgeSize = bounds.size;
        SetAxis(ref bridgeSize, longAxis, bridgeThickness);
        SetAxis(ref bridgeSize, thinAxis, GetAxis(bridgeSize, thinAxis) * 0.85f);
        float bridgeOffset = -(GetAxis(bounds.size, longAxis) - bridgeThickness) * 0.5f;
        CreateClawBox(container.transform, "Jaw_Bridge", bounds.center, bridgeSize, longAxis, bridgeOffset);
    }

    private static Renderer FindOpenClawRenderer(GameObject robot)
    {
        foreach (Renderer renderer in robot.GetComponentsInChildren<Renderer>(true))
        {
            bool soleClawMaterial = renderer.sharedMaterials.Length == 1 &&
                renderer.sharedMaterials[0] != null &&
                renderer.sharedMaterials[0].name.StartsWith("Claw_", StringComparison.OrdinalIgnoreCase);
            if (soleClawMaterial && renderer.gameObject.name == "Circle.005") return renderer;
        }
        return null;
    }

    private static void CreateClawBox(Transform parent, string name, Vector3 center, Vector3 size,
        int offsetAxis, float offset)
    {
        GameObject item = new GameObject(name);
        item.transform.SetParent(parent, false);
        BoxCollider collider = item.AddComponent<BoxCollider>();
        Vector3 adjustedCenter = center;
        SetAxis(ref adjustedCenter, offsetAxis, GetAxis(center, offsetAxis) + offset);
        collider.center = adjustedCenter;
        collider.size = size;
        collider.isTrigger = false;
    }

    private static Bounds GetRendererLocalBounds(Renderer renderer)
    {
        MeshFilter meshFilter = renderer.GetComponent<MeshFilter>();
        if (meshFilter != null && meshFilter.sharedMesh != null)
        {
            return meshFilter.sharedMesh.bounds;
        }

        if (renderer is SkinnedMeshRenderer skinned)
        {
            return skinned.localBounds;
        }

        Vector3 center = renderer.transform.InverseTransformPoint(renderer.bounds.center);
        Vector3 size = renderer.transform.InverseTransformVector(renderer.bounds.size);
        return new Bounds(center, new Vector3(Mathf.Abs(size.x), Mathf.Abs(size.y), Mathf.Abs(size.z)));
    }

    private static int LargestAxis(Vector3 size)
    {
        if (size.x >= size.y && size.x >= size.z) return 0;
        return size.y >= size.z ? 1 : 2;
    }

    private static int SmallestAxis(Vector3 size)
    {
        if (size.x <= size.y && size.x <= size.z) return 0;
        return size.y <= size.z ? 1 : 2;
    }

    private static float GetAxis(Vector3 value, int axis)
    {
        return axis == 0 ? value.x : axis == 1 ? value.y : value.z;
    }

    private static void SetAxis(ref Vector3 value, int axis, float component)
    {
        if (axis == 0) value.x = component;
        else if (axis == 1) value.y = component;
        else value.z = component;
    }

    private static bool ContainsAny(string value, params string[] fragments)
    {
        foreach (string fragment in fragments)
        {
            if (value.Contains(fragment)) return true;
        }
        return false;
    }

    private static string GetPath(Transform item, Transform root)
    {
        string path = item.name;
        while (item.parent != null && item.parent != root)
        {
            item = item.parent;
            path = item.name + "/" + path;
        }
        return path;
    }

    private static Transform CreatePoint(Transform parent, string name, Vector3 localPosition, Vector3 localEuler)
    {
        GameObject child = GetOrCreateChild(parent, name);
        child.transform.SetLocalPositionAndRotation(localPosition, Quaternion.Euler(localEuler));
        return child.transform;
    }

    private static GameObject GetOrCreateChild(Transform parent, string name)
    {
        Transform found = parent.Find(name);
        if (found != null) return found.gameObject;
        GameObject child = new GameObject(name);
        child.transform.SetParent(parent, false);
        return child;
    }

    private static T GetOrAdd<T>(GameObject target) where T : Component
    {
        T component = target.GetComponent<T>();
        return component != null ? component : target.AddComponent<T>();
    }

    private static void EnsureTag(string tag)
    {
        UnityEngine.Object[] assets = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset");
        if (assets.Length == 0) return;
        SerializedObject manager = new SerializedObject(assets[0]);
        SerializedProperty tags = manager.FindProperty("tags");
        for (int i = 0; i < tags.arraySize; i++)
        {
            if (tags.GetArrayElementAtIndex(i).stringValue == tag) return;
        }
        tags.InsertArrayElementAtIndex(tags.arraySize);
        tags.GetArrayElementAtIndex(tags.arraySize - 1).stringValue = tag;
        manager.ApplyModifiedProperties();
    }

    private static void EnsureBuildScene()
    {
        EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(ScenePath, true) };
    }

    private static void ValidateTrainingConfig()
    {
        string yaml = File.ReadAllText("config.yaml");
        foreach (char character in yaml)
        {
            Require(character <= 127, "config.yaml must contain ASCII characters only.");
        }

        string[] requiredEntries =
        {
            "GFSX_Brain:", "trainer_type: ppo", "batch_size: 2048", "buffer_size: 40960",
            "learning_rate: 0.00025", "beta: 0.005", "epsilon: 0.2", "lambd: 0.95",
            "num_epoch: 3", "learning_rate_schedule: linear", "normalize: true",
            "hidden_units: 256", "num_layers: 2", "sequence_length: 64", "memory_size: 256",
            "gamma: 0.99", "strength: 1.0", "keep_checkpoints: 5", "max_steps: 5000000",
            "time_horizon: 1000", "summary_freq: 20000"
        };
        foreach (string entry in requiredEntries)
        {
            Require(yaml.Contains(entry), $"config.yaml is missing required P3 entry: {entry}");
        }
    }

    private static void Require(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException("P2/P3 validation failed: " + message);
    }
}
