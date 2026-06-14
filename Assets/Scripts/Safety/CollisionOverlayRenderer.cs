using System;
using System.Collections.Generic;
using RoadheaderSandbox.Core.Math;
using RoadheaderSandbox.Physics;
using RoadheaderSandbox.Robotics;
using UnityEngine;

namespace RoadheaderSandbox.Safety
{
    [Serializable]
    public struct ConflictZone
    {
        public int id;
        public Vector3d worldCenter;
        public double worldRadius;
        public double compressiveStrength;
        public string materialTag;
        public double conflictStartTime;
        public double lastUpdateTime;
        public double flashIntensity;
        public bool isActive;

        public ConflictZone(int i, Vector3d c, double r, double s, string tag)
        {
            id = i;
            worldCenter = c;
            worldRadius = r;
            compressiveStrength = s;
            materialTag = tag;
            conflictStartTime = 0;
            lastUpdateTime = 0;
            flashIntensity = 0;
            isActive = false;
        }
    }

    [Serializable]
    public struct RailwayTrackSegment
    {
        public int id;
        public Vector3d start;
        public Vector3d end;
        public double gauge;
        public double length;
        public bool isInConflictZone;
        public double conflictHeat;

        public RailwayTrackSegment(int i, Vector3d s, Vector3d e, double g = 1.435)
        {
            id = i;
            start = s;
            end = e;
            gauge = g;
            length = (e - s).Magnitude;
            isInConflictZone = false;
            conflictHeat = 0;
        }
    }

    [Serializable]
    public class CollisionOverlayRenderer : MonoBehaviour
    {
        [Header("系统引用")]
        public TCPSpaceEnvelope envelope;
        public KineticFeedforwardGuard guard;
        public HydraulicSpeedLimiter limiter;
        public RoadheaderDynamics dynamics;

        [Header("高亮显示选项")]
        public bool enableOverlay = true;
        public bool drawConflictZones = true;
        public bool drawRailwayTracks = true;
        public bool drawTCPTrajectory = true;
        public bool drawEnvelope = true;
        public bool drawDeadlockLines = true;
        public bool drawHUD = true;

        [Header("冲突区域爆闪参数")]
        public double flashFrequency = 12.0;
        public double flashDuration = 3.0;
        public float outerGlowSize = 1.5f;
        public float coreGlowSize = 1.0f;

        [Header("铁轨拓扑")]
        [Tooltip("铁轨轨距 (m)，标准轨1.435")]
        public double railwayGauge = 1.435;

        [Tooltip("铁轨单段长度 (m)")]
        public double railSegmentLength = 2.0;

        [Tooltip("铁轨总长度 (m)")]
        public double totalRailLength = 30.0;

        public List<RailwayTrackSegment> railSegments = new List<RailwayTrackSegment>();
        public List<ConflictZone> activeZones = new List<ConflictZone>();
        public List<Vector3d> trajectoryHistory = new List<Vector3d>(500);
        public int maxTrajectoryPoints = 500;

        [Header("材质引用")]
        public Material glowMaterial;
        public Material railMaterial;
        public Material warningMaterial;

        [Header("诊断")]
        public int activeConflictCount;
        public double totalHeatLevel;
        public int segmentsInConflict;
        public bool deadlockActive;
        public Vector3d deadlockPoint;

        public event Action<ConflictZone> OnZoneActivated;
        public event Action<ConflictZone> OnZoneDeactivated;
        public event Action<RailwayTrackSegment> OnRailSegmentHeatChanged;
        public event Action<bool, Vector3d> OnDeadlockStateChanged;

        private GameObject _overlayRoot;
        private GameObject _railObject;
        private GameObject _zoneObject;
        private List<GameObject> _zoneMeshes = new List<GameObject>();
        private List<GameObject> _railMeshes = new List<GameObject>();
        private LineRenderer _trajectoryLine;
        private LineRenderer _deadlockLine;
        private MeshFilter _trajectoryMesh;
        private double _lastFlashTime;
        private bool _initialized;

        private void Awake()
        {
            if (envelope == null) envelope = FindObjectOfType<TCPSpaceEnvelope>();
            if (guard == null) guard = FindObjectOfType<KineticFeedforwardGuard>();
            if (limiter == null) limiter = FindObjectOfType<HydraulicSpeedLimiter>();
            if (dynamics == null) dynamics = FindObjectOfType<RoadheaderDynamics>();
        }

        private void Start()
        {
            InitializeOverlay();
            GenerateRailwayTracks();
            RegisterEvents();
            _initialized = true;
        }

        private void OnDestroy()
        {
            UnregisterEvents();
        }

        private void InitializeOverlay()
        {
            _overlayRoot = new GameObject("SafetyOverlay_Root");
            _overlayRoot.transform.SetParent(transform, false);

            _railObject = new GameObject("RailwayTracks");
            _railObject.transform.SetParent(_overlayRoot.transform, false);

            _zoneObject = new GameObject("ConflictZones");
            _zoneObject.transform.SetParent(_overlayRoot.transform, false);

            _trajectoryLine = new GameObject("TCPTrajectory").AddComponent<LineRenderer>();
            _trajectoryLine.transform.SetParent(_overlayRoot.transform, false);
            _trajectoryLine.useWorldSpace = true;
            _trajectoryLine.startWidth = 0.02f;
            _trajectoryLine.endWidth = 0.06f;
            _trajectoryLine.material = new Material(Shader.Find("Sprites/Default"));
            _trajectoryLine.startColor = new Color(0, 1, 0.5f, 0.8f);
            _trajectoryLine.endColor = new Color(0, 1, 0, 0.1f);
            _trajectoryLine.positionCount = 0;

            _deadlockLine = new GameObject("DeadlockLine").AddComponent<LineRenderer>();
            _deadlockLine.transform.SetParent(_overlayRoot.transform, false);
            _deadlockLine.useWorldSpace = true;
            _deadlockLine.startWidth = 0.05f;
            _deadlockLine.endWidth = 0.05f;
            _deadlockLine.material = new Material(Shader.Find("Sprites/Default"));
            _deadlockLine.startColor = Color.red;
            _deadlockLine.endColor = new Color(1, 0, 0, 0.5f);
            _deadlockLine.positionCount = 0;
            _deadlockLine.enabled = false;
        }

        public void GenerateRailwayTracks()
        {
            railSegments.Clear();
            int n = Mathd.Max(2, (int)(totalRailLength / railSegmentLength));
            double segLen = totalRailLength / Mathd.Max(1, n);
            Vector3d forward = Vector3d.Forward;

            for (int i = 0; i < n; i++)
            {
                Vector3d s = new Vector3d(0, 0, i * segLen - totalRailLength * 0.2);
                Vector3d e = s + forward * segLen;
                railSegments.Add(new RailwayTrackSegment(i, s, e, railwayGauge));
            }

            RebuildRailMeshes();
        }

        private void RebuildRailMeshes()
        {
            foreach (var go in _railMeshes) if (go) Destroy(go);
            _railMeshes.Clear();

            if (_railObject == null) return;

            for (int i = 0; i < railSegments.Count; i++)
            {
                CreateRailPairMesh(railSegments[i], i);
            }
        }

        private void CreateRailPairMesh(RailwayTrackSegment seg, int index)
        {
            Vector3d half = new Vector3d(seg.gauge * 0.5, 0, 0);
            for (int side = -1; side <= 1; side += 2)
            {
                GameObject rail = new GameObject($"Rail_{index}_{(side == -1 ? "L" : "R")}");
                rail.transform.SetParent(_railObject.transform, false);

                Vector3d p0 = seg.start + half * side;
                Vector3d p1 = seg.end + half * side;
                Vector3d mid = (p0 + p1) * 0.5;
                Vector3d dir = (p1 - p0).Normalized;

                rail.transform.position = mid.ToVector3();
                rail.transform.rotation = Quaternion.LookRotation(dir.ToVector3(), Vector3.up);

                BoxCollider bc = rail.AddComponent<BoxCollider>();
                bc.size = new Vector3(0.08f, 0.12f, (float)seg.length);
                bc.isTrigger = true;

                MeshFilter mf = rail.AddComponent<MeshFilter>();
                MeshRenderer mr = rail.AddComponent<MeshRenderer>();
                Mesh m = new Mesh();
                mf.mesh = m;
                m.vertices = new Vector3[]
                {
                    new Vector3(-0.04f, -0.06f, -(float)(seg.length*0.5)),
                    new Vector3( 0.04f, -0.06f, -(float)(seg.length*0.5)),
                    new Vector3( 0.04f,  0.06f, -(float)(seg.length*0.5)),
                    new Vector3(-0.04f,  0.06f, -(float)(seg.length*0.5)),
                    new Vector3(-0.04f, -0.06f,  (float)(seg.length*0.5)),
                    new Vector3( 0.04f, -0.06f,  (float)(seg.length*0.5)),
                    new Vector3( 0.04f,  0.06f,  (float)(seg.length*0.5)),
                    new Vector3(-0.04f,  0.06f,  (float)(seg.length*0.5)),
                };
                m.triangles = new int[]
                {
                    0,1,2, 0,2,3,
                    4,6,5, 4,7,6,
                    0,4,5, 0,5,1,
                    3,2,6, 3,6,7,
                    1,5,6, 1,6,2,
                    0,3,7, 0,7,4
                };
                m.RecalculateNormals();
                mr.material = railMaterial != null ? railMaterial : new Material(Shader.Find("Standard"));
                mr.material.color = new Color(0.3f, 0.3f, 0.35f, 0.9f);

                _railMeshes.Add(rail);
            }
        }

        private void RegisterEvents()
        {
            if (guard != null)
            {
                guard.OnPathIntercepted += HandleInterception;
                guard.OnGuardStateChanged += HandleGuardStateChanged;
            }
            if (envelope != null)
            {
                envelope.OnCollisionPredicted += HandleCollisionPredicted;
            }
        }

        private void UnregisterEvents()
        {
            if (guard != null)
            {
                guard.OnPathIntercepted -= HandleInterception;
                guard.OnGuardStateChanged -= HandleGuardStateChanged;
            }
            if (envelope != null)
            {
                envelope.OnCollisionPredicted -= HandleCollisionPredicted;
            }
        }

        private void HandleInterception(PathInterceptionEvent ev)
        {
            ActivateConflictZone(ev.boundaryId, ev.dangerPoint, 1.0, 300e6, ev.reason);
            UpdateRailHeatNearPoint(ev.dangerPoint, 1.0);
        }

        private void HandleGuardStateChanged(GuardState s)
        {
            bool newDeadlock = s.level >= GuardAlertLevel.Intercepted;
            Vector3d dp = s.level >= GuardAlertLevel.Warning ? s.tcpPosition : Vector3d.Zero;
            if (newDeadlock != deadlockActive)
            {
                deadlockActive = newDeadlock;
                deadlockPoint = dp;
                OnDeadlockStateChanged?.Invoke(deadlockActive, deadlockPoint);
                if (_deadlockLine != null) _deadlockLine.enabled = deadlockActive;
            }
        }

        private void HandleCollisionPredicted(CollisionPrediction p)
        {
            if (p.willCollide)
            {
                ActivateConflictZone(p.boundaryId, p.collisionPoint, 1.2, p.targetHardness,
                    p.targetHardness > 250e6 ? "预判:高硬度" : "预判:过载");
            }
        }

        public void ActivateConflictZone(int boundaryId, Vector3d center, double radius, double strength, string tag)
        {
            ConflictZone z = new ConflictZone(boundaryId, center, radius, strength, tag);
            z.conflictStartTime = Time.time;
            z.lastUpdateTime = Time.time;
            z.flashIntensity = 1.0;
            z.isActive = true;

            int existingIdx = activeZones.FindIndex(x => x.id == boundaryId);
            if (existingIdx >= 0)
            {
                z = activeZones[existingIdx];
                z.worldCenter = center;
                z.worldRadius = radius;
                z.compressiveStrength = strength;
                z.materialTag = tag;
                z.lastUpdateTime = Time.time;
                z.flashIntensity = 1.0;
                z.isActive = true;
                activeZones[existingIdx] = z;
            }
            else
            {
                activeZones.Add(z);
                OnZoneActivated?.Invoke(z);
            }

            activeConflictCount = activeZones.FindAll(x => x.isActive).Count;
            UpdateRailHeatNearPoint(center, 0.8);
        }

        public void DeactivateConflictZone(int id)
        {
            int idx = activeZones.FindIndex(x => x.id == id);
            if (idx >= 0)
            {
                ConflictZone z = activeZones[idx];
                z.isActive = false;
                activeZones[idx] = z;
                OnZoneDeactivated?.Invoke(z);
            }
            activeConflictCount = activeZones.FindAll(x => x.isActive).Count;
        }

        public void UpdateRailHeatNearPoint(Vector3d worldPos, double intensity)
        {
            double heatTotal = 0;
            int inConflict = 0;
            for (int i = 0; i < railSegments.Count; i++)
            {
                RailwayTrackSegment s = railSegments[i];
                Vector3d mid = (s.start + s.end) * 0.5;
                double dist = (mid - worldPos).Magnitude;
                double influenceRadius = s.worldRadius;
                double heat = Mathd.Max(0, 1.0 - dist / Mathd.Max(1.0, influenceRadius)) * intensity;
                s.conflictHeat = Mathd.Max(s.conflictHeat, heat);
                s.conflictHeat *= 0.99;
                s.isInConflictZone = s.conflictHeat > 0.1;
                if (s.isInConflictZone) inConflict++;
                heatTotal += s.conflictHeat;

                if (i < _railMeshes.Count)
                {
                    int meshIdx = i * 2;
                    for (int side = 0; side < 2 && meshIdx + side < _railMeshes.Count; side++)
                    {
                        GameObject rm = _railMeshes[meshIdx + side];
                        if (rm != null && rm.TryGetComponent(out MeshRenderer mr))
                        {
                            Color c = Color.Lerp(new Color(0.3f, 0.3f, 0.35f, 0.9f),
                                                 new Color(1, (float)(1.0 - s.conflictHeat), 0, 1f),
                                                 (float)s.conflictHeat);
                            mr.material.color = c;
                        }
                    }
                }
                OnRailSegmentHeatChanged?.Invoke(s);
                railSegments[i] = s;
            }
            totalHeatLevel = heatTotal;
            segmentsInConflict = inConflict;
        }

        public void AddTrajectoryPoint(Vector3d worldPos)
        {
            if (trajectoryHistory.Count >= maxTrajectoryPoints)
                trajectoryHistory.RemoveAt(0);
            trajectoryHistory.Add(worldPos);

            if (_trajectoryLine != null)
            {
                _trajectoryLine.positionCount = trajectoryHistory.Count;
                Vector3[] arr = new Vector3[trajectoryHistory.Count];
                for (int i = 0; i < trajectoryHistory.Count; i++)
                    arr[i] = trajectoryHistory[i].ToVector3();
                _trajectoryLine.SetPositions(arr);
            }
        }

        private void Update()
        {
            if (!enableOverlay || !_initialized) return;

            double now = Time.time;

            if (drawConflictZones)
                RenderConflictZones(now);

            if (drawDeadlockLines && deadlockActive && guard != null)
                RenderDeadlockLine();

            if (dynamics != null)
                AddTrajectoryPoint(dynamics.bodyState != null
                    ? dynamics.bodyState.position + new Vector3d(0, 1.0, 2.0)
                    : transform.position);

            for (int i = activeZones.Count - 1; i >= 0; i--)
            {
                ConflictZone z = activeZones[i];
                double age = now - z.lastUpdateTime;
                if (age > flashDuration)
                {
                    z.isActive = false;
                    z.flashIntensity = Mathd.Max(0, z.flashIntensity - Time.deltaTime / flashDuration);
                    if (z.flashIntensity <= 0.001)
                        activeZones.RemoveAt(i);
                    else
                        activeZones[i] = z;
                }
            }
            activeConflictCount = activeZones.FindAll(x => x.isActive).Count;
        }

        private void RenderConflictZones(double time)
        {
            int targetCount = activeZones.Count;
            while (_zoneMeshes.Count < targetCount * 2)
            {
                GameObject go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                if (go.TryGetComponent(out Collider cc)) Destroy(cc);
                go.transform.SetParent(_zoneObject.transform, false);
                MeshRenderer mr = go.GetComponent<MeshRenderer>();
                mr.material = new Material(Shader.Find("Standard"));
                mr.material.SetFloat("_Mode", 3);
                mr.material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
                mr.material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
                mr.material.EnableKeyword("_ALPHABLEND_ON");
                _zoneMeshes.Add(go);
            }

            double pulse = 0.6 + 0.4 * Mathd.Sin(time * flashFrequency * Mathd.PI * 2);

            for (int i = 0; i < activeZones.Count; i++)
            {
                ConflictZone z = activeZones[i];
                if (!z.isActive) continue;

                int coreIdx = i * 2;
                int glowIdx = i * 2 + 1;

                if (coreIdx < _zoneMeshes.Count)
                {
                    GameObject core = _zoneMeshes[coreIdx];
                    core.SetActive(true);
                    core.transform.position = z.worldCenter.ToVector3();
                    core.transform.localScale = Vector3.one * (float)z.worldRadius * 2 * (float)coreGlowSize;
                    float ratio = (float)Mathd.Clamp01(z.compressiveStrength / 400e6);
                    Color coreColor = Color.Lerp(new Color(1, 0.9f, 0, 0.35f),
                                                  new Color(1, 0.1f, 0.2f, 0.75f), ratio);
                    coreColor.a *= (float)(z.flashIntensity * pulse);
                    if (core.TryGetComponent(out MeshRenderer mr))
                    {
                        mr.material.color = coreColor;
                        mr.material.SetColor("_EmissionColor", coreColor * 2);
                    }
                    activeZones[i] = z;
                }

                if (glowIdx < _zoneMeshes.Count)
                {
                    GameObject glow = _zoneMeshes[glowIdx];
                    glow.SetActive(true);
                    glow.transform.position = z.worldCenter.ToVector3();
                    glow.transform.localScale = Vector3.one * (float)z.worldRadius * 2 * (float)outerGlowSize;
                    float ratio = (float)Mathd.Clamp01(z.compressiveStrength / 400e6);
                    Color glowColor = Color.Lerp(new Color(1, 0.9f, 0, 0.2f),
                                                  new Color(1, 0, 0.3f, 0.5f), ratio);
                    glowColor.a *= (float)(z.flashIntensity * pulse * 0.6);
                    if (glow.TryGetComponent(out MeshRenderer mr))
                    {
                        mr.material.color = glowColor;
                        mr.material.SetColor("_EmissionColor", glowColor);
                    }
                }
            }

            for (int i = activeZones.Count * 2; i < _zoneMeshes.Count; i++)
                if (_zoneMeshes[i]) _zoneMeshes[i].SetActive(false);
        }

        private void RenderDeadlockLine()
        {
            if (_deadlockLine == null || guard == null || envelope == null) return;

            Vector3d tcp = guard.currentState.tcpPosition;
            CollisionPrediction p = envelope.latestPrediction;
            if (!p.willCollide) return;

            _deadlockLine.positionCount = 2;
            _deadlockLine.SetPosition(0, tcp.ToVector3());
            _deadlockLine.SetPosition(1, p.collisionPoint.ToVector3());
            float pulse = 0.6f + 0.4f * Mathf.Sin(Time.time * 15.0f);
            _deadlockLine.startColor = new Color(1, pulse * 0.2f, 0.1f, pulse);
            _deadlockLine.endColor = new Color(1, 0, 0, pulse * 0.7f);
        }

        private void OnGUI()
        {
            if (!drawHUD) return;

            int w = 320, h = 200;
            int x = Screen.width - w - 15, y = 200;
            GUILayout.BeginArea(new Rect(x, y, w, h), "安全防护系统 - HUD", "window");

            GUILayout.Label($"防护等级: {GetLevelText(guard != null ? guard.displayLevel : GuardAlertLevel.Normal)}");
            GUILayout.Label($"主动限速系数: {(guard != null ? guard.currentSpeedRatio : 1.0):F3}x");
            GUILayout.Label($"生效限速系数: {(limiter != null ? limiter.effectiveSpeedRatio : 1.0):F3}x");
            GUILayout.Label($"急停状态: {(limiter != null && limiter.isEmergencyBraking ? "✅ 生效" : "未触发")}");
            GUILayout.Label($"路径拦截次数: {guard != null ? guard.totalInterceptions : 0}");
            GUILayout.Label($"冲突死锁区域: {activeConflictCount}");
            GUILayout.Label($"铁轨受热段数: {segmentsInConflict}/{railSegments.Count}");

            if (guard != null && guard.currentState.level >= GuardAlertLevel.Warning)
            {
                Color old = GUI.color;
                GUI.color = new Color(1, 0.2f + 0.6f * Mathf.Abs(Mathf.Sin(Time.time * 6)), 0, 1);
                GUILayout.Label($"⚠ 警报: {guard.currentState.message}");
                GUI.color = old;
            }

            GUILayout.EndArea();
        }

        private string GetLevelText(GuardAlertLevel level)
        {
            switch (level)
            {
                case GuardAlertLevel.Normal: return "<color=#00FF88>● 正常 (Green)</color>";
                case GuardAlertLevel.Warning: return "<color=#FFCC00>● 预警 (Yellow)</color>";
                case GuardAlertLevel.Critical: return "<color=#FF7700>● 临界 (Orange)</color>";
                case GuardAlertLevel.EmergencyStop: return "<color=#FF2244>● 急停 (Red)</color>";
                case GuardAlertLevel.Intercepted: return "<color=#FF0000>● 路径拦截 (Red Flash)</color>";
                default: return "未知";
            }
        }
    }
}
