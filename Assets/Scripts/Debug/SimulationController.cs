using System;
using System.Collections.Generic;
using RoadheaderSandbox.Core.Math;
using RoadheaderSandbox.Kinematics;
using RoadheaderSandbox.Physics;
using RoadheaderSandbox.Robotics;
using UnityEngine;

namespace RoadheaderSandbox.Debug
{
    [Serializable]
    public class SimulationController : MonoBehaviour
    {
        [Header("系统引用")]
        public RoadwayProfileGenerator profileGenerator;
        public CuttingHeadMotionController cuttingHeadController;
        public ArmSkeletonController armController;
        public CuttingHeadCollisionSolver collisionSolver;
        public RoadheaderDynamics dynamics;
        public RockSurface rockSurface;

        [Header("仿真设置")]
        public bool autoStart = true;
        public double initialFeedDepth = 3.0;
        public double headSpinSpeed = 10.0;
        public double traverseSpeed = 0.3;

        [Header("截割齿配置")]
        public int pickCountPerRow = 6;
        public int pickRowCount = 3;
        public double pickTipRadius = 0.008;
        public double pickRadialPosition = 0.4;
        public double pickInstallationAngle = 45.0;

        [Header("记录数据")]
        public List<double> timeHistory = new List<double>();
        public List<double> forceHistory = new List<double>();
        public List<double> torqueHistory = new List<double>();
        public List<double> volumeHistory = new List<double>();
        public int maxHistoryLength = 1000;

        [Header("控制")]
        public bool isRunning;
        public double simulationTime;

        public event Action OnSimulationStarted;
        public event Action OnSimulationStopped;
        public event Action<double> OnSimulationStep;

        private bool _initialized;

        private void Awake()
        {
            Initialize();
        }

        private void Start()
        {
            if (autoStart)
            {
                StartSimulation();
            }
        }

        public void Initialize()
        {
            if (_initialized) return;

            if (profileGenerator == null)
                profileGenerator = FindObjectOfType<RoadwayProfileGenerator>();

            if (cuttingHeadController == null)
                cuttingHeadController = FindObjectOfType<CuttingHeadMotionController>();

            if (armController == null)
                armController = FindObjectOfType<ArmSkeletonController>();

            if (collisionSolver == null)
                collisionSolver = FindObjectOfType<CuttingHeadCollisionSolver>();

            if (dynamics == null)
                dynamics = FindObjectOfType<RoadheaderDynamics>();

            if (rockSurface == null)
                rockSurface = FindObjectOfType<RockSurface>();

            if (profileGenerator != null)
            {
                profileGenerator.GenerateProfile();
                profileGenerator.GenerateCuttingPath(initialFeedDepth);
            }

            if (cuttingHeadController != null && profileGenerator != null)
            {
                cuttingHeadController.SetSpline(profileGenerator.CuttingPathSpline);
                cuttingHeadController.spinSpeed = headSpinSpeed;
                cuttingHeadController.traverseSpeed = traverseSpeed;
            }

            GeneratePicks();

            _initialized = true;
        }

        public void GeneratePicks()
        {
            if (collisionSolver == null) return;

            Transform headTransform = collisionSolver.transform;

            for (int i = headTransform.childCount - 1; i >= 0; i--)
            {
                if (headTransform.GetChild(i).GetComponent<Pick>() != null)
                {
                    DestroyImmediate(headTransform.GetChild(i).gameObject);
                }
            }

            collisionSolver.picks.Clear();

            for (int row = 0; row < pickRowCount; row++)
            {
                double axialPos = (row - (pickRowCount - 1) * 0.5) * 0.15;

                for (int i = 0; i < pickCountPerRow; i++)
                {
                    double circumAngle = (double)i / pickCountPerRow * 360.0 + row * 15.0;

                    GameObject pickObj = new GameObject($"Pick_Row{row}_{i}");
                    pickObj.transform.SetParent(headTransform, false);

                    Pick pick = pickObj.AddComponent<Pick>();
                    pick.tipRadius = pickTipRadius;
                    pick.bodyRadius = pickTipRadius * 1.5;
                    pick.totalLength = pickTipRadius * 10;
                    pick.tipAngle = 75.0;
                    pick.installationAngle = pickInstallationAngle;
                    pick.circumferentialAngle = circumAngle;
                    pick.axialPosition = axialPos;
                    pick.radialPosition = pickRadialPosition;
                    pick.material = MaterialProperties.TungstenCarbide();

                    collisionSolver.AddPick(pick);
                }
            }

            collisionSolver.headRadius = pickRadialPosition + pickTipRadius;
            collisionSolver.headLength = pickRowCount * 0.15 + 0.3;
        }

        public void StartSimulation()
        {
            if (!_initialized) Initialize();

            isRunning = true;
            simulationTime = 0;

            if (cuttingHeadController != null)
            {
                cuttingHeadController.StartMotion();
                cuttingHeadController.StartSpin();
            }

            OnSimulationStarted?.Invoke();
        }

        public void StopSimulation()
        {
            isRunning = false;

            if (cuttingHeadController != null)
            {
                cuttingHeadController.StopMotion();
                cuttingHeadController.StopSpin();
            }

            OnSimulationStopped?.Invoke();
        }

        public void ResetSimulation()
        {
            StopSimulation();

            simulationTime = 0;
            timeHistory.Clear();
            forceHistory.Clear();
            torqueHistory.Clear();
            volumeHistory.Clear();

            if (dynamics != null)
            {
                dynamics.ResetDynamics();
            }

            if (cuttingHeadController != null)
            {
                cuttingHeadController.ResetMotion();
            }

            if (profileGenerator != null)
            {
                profileGenerator.GenerateProfile();
                profileGenerator.GenerateCuttingPath(initialFeedDepth);
                cuttingHeadController.SetSpline(profileGenerator.CuttingPathSpline);
            }

            GeneratePicks();
        }

        private void Update()
        {
            if (!isRunning) return;

            simulationTime += Time.deltaTime;

            RecordData();

            OnSimulationStep?.Invoke(simulationTime);
        }

        private void RecordData()
        {
            if (collisionSolver == null) return;

            if (timeHistory.Count >= maxHistoryLength)
            {
                timeHistory.RemoveAt(0);
                forceHistory.RemoveAt(0);
                torqueHistory.RemoveAt(0);
                volumeHistory.RemoveAt(0);
            }

            timeHistory.Add(simulationTime);
            forceHistory.Add(collisionSolver.totalContactForce.Magnitude);
            torqueHistory.Add(collisionSolver.totalContactTorque.Magnitude);
            volumeHistory.Add(collisionSolver.totalExcavationVolume);
        }

        public string GetReport()
        {
            if (collisionSolver == null || dynamics == null) return "Simulation not initialized";

            double avgForce = forceHistory.Count > 0 ? forceHistory[forceHistory.Count - 1] : 0;
            double avgTorque = torqueHistory.Count > 0 ? torqueHistory[torqueHistory.Count - 1] : 0;
            double totalVolume = volumeHistory.Count > 0 ? volumeHistory[volumeHistory.Count - 1] : 0;
            double productionRate = simulationTime > 0 ? totalVolume / simulationTime * 3600 : 0;
            double specificEnergy = collisionSolver.specificEnergy / 1e6;

            return string.Format(
                "=== 掘进机仿真报告 ===\n" +
                "仿真时间: {0:F2} s\n" +
                "当前速度: {1:F3} m/s\n" +
                "推进速度: {2:F2} m/h\n" +
                "接触齿数: {3}\n" +
                "总接触力: {4:F2} kN\n" +
                "总接触力矩: {5:F2} kN·m\n" +
                "平均接触力: {6:F2} kN\n" +
                "最大接触力: {7:F2} kN\n" +
                "总开挖量: {8:F4} m³\n" +
                "生产率: {9:F2} m³/h\n" +
                "比能耗: {10:F3} MJ/m³\n" +
                "截割头转速: {11:F1} rad/s\n" +
                "截割头位置: {12}\n" +
                "=====================",
                simulationTime,
                dynamics.currentSpeed,
                dynamics.currentAdvanceRate,
                collisionSolver.activeContactCount,
                avgForce / 1000.0,
                avgTorque / 1000.0,
                collisionSolver.GetAverageContactForce() / 1000.0,
                collisionSolver.GetMaximumContactForce() / 1000.0,
                totalVolume,
                productionRate,
                specificEnergy,
                cuttingHeadController != null ? cuttingHeadController.spinSpeed : 0,
                cuttingHeadController != null ? cuttingHeadController.SmoothedPosition.ToString() : "N/A"
            );
        }

        private void OnGUI()
        {
            if (!isRunning) return;

            GUILayout.BeginArea(new Rect(10, 10, 350, 300));
            GUILayout.BeginVertical("box");

            GUILayout.Label("悬臂式掘进机 ATO 仿真沙盒", GUILayout.Width(300));
            GUILayout.Label(new string('-', 40));
            GUILayout.Label(GetReport());

            if (GUILayout.Button("停止仿真", GUILayout.Width(150)))
            {
                StopSimulation();
            }

            if (GUILayout.Button("重置仿真", GUILayout.Width(150)))
            {
                ResetSimulation();
            }

            GUILayout.EndVertical();
            GUILayout.EndArea();

            DrawForceGraph();
        }

        private void DrawForceGraph()
        {
            if (forceHistory.Count < 2) return;

            int width = 400;
            int height = 150;
            int margin = 10;

            Rect graphRect = new Rect(Screen.width - width - 10, 10, width, height);

            GUILayout.BeginArea(graphRect);
            GUILayout.BeginVertical("box");
            GUILayout.Label("接触力曲线 (kN)");

            Texture2D tex = new Texture2D(width, height);
            Color bg = new Color(0.1f, 0.1f, 0.1f, 0.8f);
            for (int x = 0; x < width; x++)
                for (int y = 0; y < height; y++)
                    tex.SetPixel(x, y, bg);

            double maxForce = 0;
            for (int i = 0; i < forceHistory.Count; i++)
            {
                if (forceHistory[i] > maxForce) maxForce = forceHistory[i];
            }
            maxForce = Mathd.Max(maxForce / 1000.0, 10);

            for (int i = 1; i < forceHistory.Count; i++)
            {
                int x1 = margin + (i - 1) * (width - 2 * margin) / Math.Max(forceHistory.Count - 1, 1);
                int x2 = margin + i * (width - 2 * margin) / Math.Max(forceHistory.Count - 1, 1);
                int y1 = height - margin - (int)(forceHistory[i - 1] / 1000.0 / maxForce * (height - 2 * margin));
                int y2 = height - margin - (int)(forceHistory[i] / 1000.0 / maxForce * (height - 2 * margin));

                DrawLine(tex, x1, y1, x2, y2, Color.red);
            }

            tex.Apply();
            GUILayout.Box(tex);
            GUILayout.Label(string.Format("最大: {0:F1} kN", maxForce));

            GUILayout.EndVertical();
            GUILayout.EndArea();
        }

        private void DrawLine(Texture2D tex, int x1, int y1, int x2, int y2, Color color)
        {
            int dx = Math.Abs(x2 - x1);
            int dy = Math.Abs(y2 - y1);
            int sx = x1 < x2 ? 1 : -1;
            int sy = y1 < y2 ? 1 : -1;
            int err = dx - dy;

            while (true)
            {
                if (x1 >= 0 && x1 < tex.width && y1 >= 0 && y1 < tex.height)
                    tex.SetPixel(x1, y1, color);

                if (x1 == x2 && y1 == y2) break;

                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; x1 += sx; }
                if (e2 < dx) { err += dx; y1 += sy; }
            }
        }
    }
}
