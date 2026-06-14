using UnityEngine;
using RoadheaderSandbox.Kinematics;
using RoadheaderSandbox.Physics;
using RoadheaderSandbox.Robotics;
using RoadheaderSandbox.Debug;

namespace RoadheaderSandbox.Debug
{
    public class SceneBuilder : MonoBehaviour
    {
        public static SceneBuilder Instance { get; private set; }

        [Header("场景根节点")]
        public Transform sceneRoot;

        [Header("生成的对象")]
        public RoadwayProfileGenerator profileGenerator;
        public RockSurface rockSurface;
        public CuttingHeadCollisionSolver collisionSolver;
        public CuttingHeadMotionController cuttingHeadController;
        public ArmSkeletonController armController;
        public RoadheaderDynamics roadheaderDynamics;
        public SimulationController simulationController;

        public Transform roadheaderBody;
        public Transform cuttingHead;
        public Transform armBase;
        public Transform armSegment1;
        public Transform armSegment2;

        private void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(gameObject);
                return;
            }
            Instance = this;
        }

        [ContextMenu("Build Complete Scene")]
        public void BuildCompleteScene()
        {
            ClearScene();
            CreateSceneRoot();
            CreateRockSurface();
            CreateRoadwayProfile();
            CreateRoadheaderBody();
            CreateArmSkeleton();
            CreateCuttingHead();
            CreateCollisionSolver();
            CreateMotionController();
            CreateDynamics();
            CreateSimulationController();

            Debug.Log("Scene build complete!");
        }

        [ContextMenu("Clear Scene")]
        public void ClearScene()
        {
            for (int i = transform.childCount - 1; i >= 0; i--)
            {
                DestroyImmediate(transform.GetChild(i).gameObject);
            }

            profileGenerator = null;
            rockSurface = null;
            collisionSolver = null;
            cuttingHeadController = null;
            armController = null;
            roadheaderDynamics = null;
            simulationController = null;
            roadheaderBody = null;
            cuttingHead = null;
            armBase = null;
            armSegment1 = null;
            armSegment2 = null;
        }

        private void CreateSceneRoot()
        {
            GameObject rootObj = new GameObject("SceneRoot");
            rootObj.transform.SetParent(transform, false);
            sceneRoot = rootObj.transform;
        }

        private void CreateRockSurface()
        {
            GameObject rockObj = new GameObject("RockSurface");
            rockObj.transform.SetParent(sceneRoot, false);
            rockObj.transform.position = new Vector3(0, 0, 5);

            rockSurface = rockObj.AddComponent<RockSurface>();
            rockSurface.width = 8.0;
            rockSurface.height = 6.0;
            rockSurface.depth = 15.0;
            rockSurface.resolution = 10.0;
            rockSurface.roughnessAmplitude = 0.03;
            rockSurface.roughnessFrequency = 4.0;
            rockSurface.material = MaterialProperties.Rock();

            GameObject rockVisual = GameObject.CreatePrimitive(PrimitiveType.Cube);
            rockVisual.name = "RockVisual";
            rockVisual.transform.SetParent(rockObj.transform, false);
            rockVisual.transform.localPosition = new Vector3(0, 0, 0);
            rockVisual.transform.localScale = new Vector3(8f, 6f, 15f);
            DestroyImmediate(rockVisual.GetComponent<Collider>());

            Renderer renderer = rockVisual.GetComponent<Renderer>();
            Material rockMat = new Material(Shader.Find("Standard"));
            rockMat.color = new Color(0.6f, 0.4f, 0.2f);
            renderer.material = rockMat;
        }

        private void CreateRoadwayProfile()
        {
            GameObject profileObj = new GameObject("RoadwayProfile");
            profileObj.transform.SetParent(sceneRoot, false);
            profileObj.transform.position = new Vector3(0, 1.75f, 3);

            profileGenerator = profileObj.AddComponent<RoadwayProfileGenerator>();
            profileGenerator.roadwayWidth = 5.0;
            profileGenerator.roadwayHeight = 3.5;
            profileGenerator.floorRadius = 0.5;
            profileGenerator.roofRadius = 2.0;
            profileGenerator.sideAngle = 5.0;
            profileGenerator.profileCenter = new Vector3(0, 0, 0);
            profileGenerator.feedStep = 0.5;
            profileGenerator.maxFeedDepth = 10.0;
            profileGenerator.samplesPerSegment = 20;
            profileGenerator.shapeType = RoadwayProfileGenerator.ProfileShapeType.Arched;

            profileGenerator.GenerateProfile();
            profileGenerator.GenerateCuttingPath(3.0);
        }

        private void CreateRoadheaderBody()
        {
            GameObject bodyObj = new GameObject("RoadheaderBody");
            bodyObj.transform.SetParent(sceneRoot, false);
            bodyObj.transform.position = new Vector3(0, 0.9f, -1);
            bodyObj.transform.rotation = Quaternion.identity;

            roadheaderBody = bodyObj.transform;

            GameObject bodyVisual = GameObject.CreatePrimitive(PrimitiveType.Cube);
            bodyVisual.name = "BodyVisual";
            bodyVisual.transform.SetParent(bodyObj.transform, false);
            bodyVisual.transform.localPosition = new Vector3(0, 0, 0);
            bodyVisual.transform.localScale = new Vector3(2.5f, 1.8f, 8.0f);
            DestroyImmediate(bodyVisual.GetComponent<Collider>());

            Renderer renderer = bodyVisual.GetComponent<Renderer>();
            Material bodyMat = new Material(Shader.Find("Standard"));
            bodyMat.color = new Color(0.3f, 0.3f, 0.35f);
            renderer.material = bodyMat;

            GameObject trackLeft = GameObject.CreatePrimitive(PrimitiveType.Cube);
            trackLeft.name = "TrackLeft";
            trackLeft.transform.SetParent(bodyObj.transform, false);
            trackLeft.transform.localPosition = new Vector3(-1.3f, -0.6f, 0);
            trackLeft.transform.localScale = new Vector3(0.4f, 0.6f, 7.5f);
            DestroyImmediate(trackLeft.GetComponent<Collider>());

            GameObject trackRight = GameObject.CreatePrimitive(PrimitiveType.Cube);
            trackRight.name = "TrackRight";
            trackRight.transform.SetParent(bodyObj.transform, false);
            trackRight.transform.localPosition = new Vector3(1.3f, -0.6f, 0);
            trackRight.transform.localScale = new Vector3(0.4f, 0.6f, 7.5f);
            DestroyImmediate(trackRight.GetComponent<Collider>());

            Material trackMat = new Material(Shader.Find("Standard"));
            trackMat.color = new Color(0.15f, 0.15f, 0.15f);
            trackLeft.GetComponent<Renderer>().material = trackMat;
            trackRight.GetComponent<Renderer>().material = trackMat;

            roadheaderDynamics = bodyObj.AddComponent<RoadheaderDynamics>();
            roadheaderDynamics.bodyFrame = bodyObj.transform;
            roadheaderDynamics.totalMass = 45000.0;
            roadheaderDynamics.machineSize = new Vector3(2.5f, 1.8f, 8.0f);
            roadheaderDynamics.centerOfMass = new Vector3(0, 0.5f, 2.0f);
            roadheaderDynamics.fixedTimestep = 0.001;
            roadheaderDynamics.maxSubSteps = 10;
            roadheaderDynamics.enableGravity = true;
        }

        private void CreateArmSkeleton()
        {
            GameObject armBaseObj = new GameObject("ArmBase");
            armBaseObj.transform.SetParent(roadheaderBody, false);
            armBaseObj.transform.localPosition = new Vector3(0, 1.0f, -2.5f);
            armBase = armBaseObj.transform;

            GameObject arm1Obj = new GameObject("ArmSegment1");
            arm1Obj.transform.SetParent(armBase, false);
            arm1Obj.transform.localPosition = new Vector3(0, 0, 1.5f);
            armSegment1 = arm1Obj.transform;

            GameObject arm2Obj = new GameObject("ArmSegment2");
            arm2Obj.transform.SetParent(armSegment1, false);
            arm2Obj.transform.localPosition = new Vector3(0, 0, 2.0f);
            armSegment2 = arm2Obj.transform;

            CreateArmVisual(armBase, new Vector3(0, 0, 0.75f), new Vector3(0.3f, 0.3f, 1.5f), new Color(0.4f, 0.4f, 0.45f));
            CreateArmVisual(armSegment1, new Vector3(0, 0, 1.0f), new Vector3(0.25f, 0.25f, 2.0f), new Color(0.35f, 0.35f, 0.4f));
            CreateArmVisual(armSegment2, new Vector3(0, 0, 1.0f), new Vector3(0.2f, 0.2f, 2.0f), new Color(0.3f, 0.3f, 0.35f));

            armController = armBaseObj.AddComponent<ArmSkeletonController>();
            armController.bodyFrame = roadheaderBody;
            armController.useBodyFrame = true;
            armController.useInverseKinematics = true;
            armController.ikIterations = 50;
            armController.ikTolerance = 0.001;
            armController.enableDynamics = true;

            armController.segments.Add(new ArmSegment
            {
                name = "ArmBase",
                transform = armBase,
                length = 1.5,
                jointAxis = Vector3d.Up,
                minAngle = -45.0 * Mathd.Deg2Rad,
                maxAngle = 45.0 * Mathd.Deg2Rad,
                maxAngularVelocity = 90.0 * Mathd.Deg2Rad,
                damping = 5.0,
                stiffness = 100.0
            });

            armController.segments.Add(new ArmSegment
            {
                name = "ArmSegment1",
                transform = armSegment1,
                length = 2.0,
                jointAxis = Vector3d.Right,
                minAngle = -30.0 * Mathd.Deg2Rad,
                maxAngle = 60.0 * Mathd.Deg2Rad,
                maxAngularVelocity = 60.0 * Mathd.Deg2Rad,
                damping = 5.0,
                stiffness = 100.0
            });

            armController.segments.Add(new ArmSegment
            {
                name = "ArmSegment2",
                transform = armSegment2,
                length = 2.0,
                jointAxis = Vector3d.Right,
                minAngle = -60.0 * Mathd.Deg2Rad,
                maxAngle = 30.0 * Mathd.Deg2Rad,
                maxAngularVelocity = 60.0 * Mathd.Deg2Rad,
                damping = 5.0,
                stiffness = 100.0
            });
        }

        private void CreateArmVisual(Transform parent, Vector3 localPos, Vector3 size, Color color)
        {
            GameObject visual = GameObject.CreatePrimitive(PrimitiveType.Cube);
            visual.name = "Visual";
            visual.transform.SetParent(parent, false);
            visual.transform.localPosition = localPos;
            visual.transform.localScale = size;
            DestroyImmediate(visual.GetComponent<Collider>());

            Renderer renderer = visual.GetComponent<Renderer>();
            Material mat = new Material(Shader.Find("Standard"));
            mat.color = color;
            renderer.material = mat;
        }

        private void CreateCuttingHead()
        {
            GameObject headObj = new GameObject("CuttingHead");
            headObj.transform.SetParent(armSegment2, false);
            headObj.transform.localPosition = new Vector3(0, 0, 2.0f);
            cuttingHead = headObj.transform;

            GameObject headVisual = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            headVisual.name = "HeadVisual";
            headVisual.transform.SetParent(cuttingHead, false);
            headVisual.transform.localScale = new Vector3(0.8f, 0.8f, 1.2f);
            DestroyImmediate(headVisual.GetComponent<Collider>());

            Renderer renderer = headVisual.GetComponent<Renderer>();
            Material headMat = new Material(Shader.Find("Standard"));
            headMat.color = new Color(0.5f, 0.5f, 0.55f);
            renderer.material = headMat;

            cuttingHeadController = headObj.AddComponent<CuttingHeadMotionController>();
            cuttingHeadController.profileGenerator = profileGenerator;
            cuttingHeadController.forwardAxis = new Vector3(0, 0, 1);
            cuttingHeadController.upAxis = new Vector3(0, 1, 0);
            cuttingHeadController.followTangent = true;
            cuttingHeadController.orientationWeight = 1.0;
            cuttingHeadController.traverseSpeed = 0.3;
            cuttingHeadController.spinSpeed = 10.0;
            cuttingHeadController.smoothingTime = 0.1;
            cuttingHeadController.maxSpeed = 2.0;
        }

        private void CreateCollisionSolver()
        {
            collisionSolver = cuttingHead.gameObject.AddComponent<CuttingHeadCollisionSolver>();
            collisionSolver.rockSurface = rockSurface;
            collisionSolver.headRadius = 0.4;
            collisionSolver.headLength = 0.8;
            collisionSolver.excavationForceThreshold = 1000.0;
            collisionSolver.excavationEfficiency = 0.7;
            collisionSolver.forceDrawScale = 0.0001;
            collisionSolver.drawContactForces = true;

            collisionSolver.contactSolver = new HertzContactSolver
            {
                maxIterations = 50,
                tolerance = 1e-12,
                useMindlinTheory = true,
                includeDamping = true,
                includePlasticity = true
            };
        }

        private void CreateMotionController()
        {
            if (armController != null)
            {
                armController.cuttingHeadController = cuttingHeadController;
                armController.endEffector = cuttingHead;
            }

            if (roadheaderDynamics != null)
            {
                roadheaderDynamics.cuttingHeadController = cuttingHeadController;
                roadheaderDynamics.armController = armController;
                roadheaderDynamics.collisionSolver = collisionSolver;
                roadheaderDynamics.rockSurface = rockSurface;
            }
        }

        private void CreateDynamics()
        {
        }

        private void CreateSimulationController()
        {
            GameObject simObj = new GameObject("SimulationController");
            simObj.transform.SetParent(sceneRoot, false);

            simulationController = simObj.AddComponent<SimulationController>();
            simulationController.profileGenerator = profileGenerator;
            simulationController.cuttingHeadController = cuttingHeadController;
            simulationController.armController = armController;
            simulationController.collisionSolver = collisionSolver;
            simulationController.dynamics = roadheaderDynamics;
            simulationController.rockSurface = rockSurface;
            simulationController.autoStart = true;
            simulationController.initialFeedDepth = 3.0;
            simulationController.headSpinSpeed = 10.0;
            simulationController.traverseSpeed = 0.3;
            simulationController.pickCountPerRow = 6;
            simulationController.pickRowCount = 3;
            simulationController.pickTipRadius = 0.008;
            simulationController.pickRadialPosition = 0.4;
            simulationController.pickInstallationAngle = 45.0;
            simulationController.maxHistoryLength = 1000;
        }

        [ContextMenu("Run Validation Checks")]
        public void RunValidationChecks()
        {
            bool allPassed = true;

            Debug.Log("=== 系统验证检查 ===");

            if (profileGenerator == null)
            {
                Debug.LogError("FAIL: RoadwayProfileGenerator not found");
                allPassed = false;
            }
            else
            {
                Debug.Log("PASS: RoadwayProfileGenerator present");
                if (profileGenerator.ProfileSpline == null)
                {
                    Debug.LogWarning("WARN: Profile spline not generated");
                }
                else
                {
                    Debug.Log($"PASS: Profile spline generated, length = {profileGenerator.ProfileSpline.TotalLength:F3}m");
                }
            }

            if (rockSurface == null)
            {
                Debug.LogError("FAIL: RockSurface not found");
                allPassed = false;
            }
            else
            {
                Debug.Log("PASS: RockSurface present");
                Debug.Log($"      Rock size: {rockSurface.width}x{rockSurface.height}x{rockSurface.depth}m");
            }

            if (cuttingHeadController == null)
            {
                Debug.LogError("FAIL: CuttingHeadMotionController not found");
                allPassed = false;
            }
            else
            {
                Debug.Log("PASS: CuttingHeadMotionController present");
                Debug.Log($"      Spin speed: {cuttingHeadController.spinSpeed} rad/s");
                Debug.Log($"      Traverse speed: {cuttingHeadController.traverseSpeed} m/s");
            }

            if (armController == null)
            {
                Debug.LogError("FAIL: ArmSkeletonController not found");
                allPassed = false;
            }
            else
            {
                Debug.Log("PASS: ArmSkeletonController present");
                Debug.Log($"      Segments: {armController.SegmentCount}");
                Debug.Log($"      Total reach: {armController.GetTotalReach():F3}m");
            }

            if (collisionSolver == null)
            {
                Debug.LogError("FAIL: CuttingHeadCollisionSolver not found");
                allPassed = false;
            }
            else
            {
                Debug.Log("PASS: CuttingHeadCollisionSolver present");
                Debug.Log($"      Picks: {collisionSolver.picks.Count}");
                Debug.Log($"      Contact solver: Mindlin={collisionSolver.contactSolver.useMindlinTheory}");
            }

            if (roadheaderDynamics == null)
            {
                Debug.LogError("FAIL: RoadheaderDynamics not found");
                allPassed = false;
            }
            else
            {
                Debug.Log("PASS: RoadheaderDynamics present");
                Debug.Log($"      Mass: {roadheaderDynamics.totalMass}kg");
                Debug.Log($"      Timestep: {roadheaderDynamics.fixedTimestep}s");
            }

            if (simulationController == null)
            {
                Debug.LogError("FAIL: SimulationController not found");
                allPassed = false;
            }
            else
            {
                Debug.Log("PASS: SimulationController present");
                Debug.Log($"      Auto start: {simulationController.autoStart}");
            }

            Debug.Log($"=== 验证结果: {(allPassed ? "全部通过" : "存在错误")} ===");
        }
    }
}
