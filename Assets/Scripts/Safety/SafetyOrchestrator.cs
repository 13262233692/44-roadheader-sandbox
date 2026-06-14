using System;
using System.Collections.Generic;
using RoadheaderSandbox.Core.Math;
using RoadheaderSandbox.Kinematics;
using RoadheaderSandbox.Physics;
using RoadheaderSandbox.Robotics;
using UnityEngine;

namespace RoadheaderSandbox.Safety
{
    [Serializable]
    public class SafetyOrchestrator : MonoBehaviour
    {
        [Header("核心安全引擎 - 6模块联动")]
        public DHParameterIK ikSolver;
        public TCPSpaceEnvelope envelope;
        public KineticFeedforwardGuard guard;
        public HydraulicSpeedLimiter limiter;
        public CollisionOverlayRenderer overlay;

        [Header("业务系统引用")]
        public ArmSkeletonController arm;
        public CuttingHeadMotionController cuttingHead;
        public RoadheaderDynamics dynamics;
        public CuttingHeadCollisionSolver collisionSolver;
        public RockSurface rockSurface;

        [Header("总控开关")]
        [Tooltip("启用生产中控级安全防护")]
        public bool enableSafetySystem = true;

        [Tooltip("启用 FixedUpdate 同步执行")]
        public bool useFixedUpdate = true;

        [Tooltip("安全周期执行频率倍率 (相对物理步长)")]
        public int subStepFactor = 1;

        [Header("TCP状态输入")]
        [Tooltip("使用动力学真实位置/速度")]
        public bool useDynamicsForTCP = true;

        [Tooltip("TCP相对机体的偏移")]
        public Vector3d tcpOffsetFromBody = new Vector3d(0, 0.8, 3.0);

        [Header("诊断统计")]
        public double lastFrameExecutionUs;
        public double averageExecutionUs;
        public long totalExecutionCalls;
        public int safetyInterventions;
        public int pathsIntercepted;
        public double cumulativeEnergySaved;

        [Header("调试")]
        public Vector3d debugTCPVelocity;
        public Vector3d debugTCPAngularVelocity;
        public double debugTCPSpeed;
        public double debugTcpKinetic;
        public GuardAlertLevel debugGuardLevel;
        public double debugSpeedRatio;
        public bool debugEmergency;

        public event Action<SafetyOrchestrator> OnSafetySystemInitialized;
        public event Action<int, double> OnSafetyIntervention;
        public event Action<Vector3d, Vector3d, double> OnTCPUpdated;

        private Vector3d _lastTCPPosition;
        private Quaterniond _lastTCPRotation;
        private double _lastTimestamp;
        private bool _initialized;
        private int _subStepCounter;

        private void Awake()
        {
            InitializeSafetySystem();
        }

        private void Start()
        {
            StartCoroutine(DelayedAttachEvents());
        }

        private System.Collections.IEnumerator DelayedAttachEvents()
        {
            yield return new WaitForEndOfFrame();
            AttachGuardCallbacks();
            yield return null;
        }

        public void InitializeSafetySystem()
        {
            if (_initialized) return;

            if (ikSolver == null)
            {
                GameObject go = new GameObject("DHParameterIK");
                go.transform.SetParent(transform, false);
                ikSolver = go.AddComponent<DHParameterIK>();
            }

            if (envelope == null)
            {
                GameObject go = new GameObject("TCPSpaceEnvelope");
                go.transform.SetParent(transform, false);
                envelope = go.AddComponent<TCPSpaceEnvelope>();
                envelope.rockSurface = rockSurface;
            }

            if (guard == null)
            {
                GameObject go = new GameObject("KineticFeedforwardGuard");
                go.transform.SetParent(transform, false);
                guard = go.AddComponent<KineticFeedforwardGuard>();
                guard.envelope = envelope;
                guard.ikSolver = ikSolver;
                guard.collisionSolver = collisionSolver;
                guard.rockSurface = rockSurface;
            }

            if (limiter == null)
            {
                GameObject go = new GameObject("HydraulicSpeedLimiter");
                go.transform.SetParent(transform, false);
                limiter = go.AddComponent<HydraulicSpeedLimiter>();
                limiter.InitializeReferences(arm, cuttingHead, dynamics, guard, ikSolver);
            }
            else
            {
                limiter.InitializeReferences(arm, cuttingHead, dynamics, guard, ikSolver);
            }

            if (overlay == null)
            {
                GameObject go = new GameObject("CollisionOverlayRenderer");
                go.transform.SetParent(transform, false);
                overlay = go.AddComponent<CollisionOverlayRenderer>();
                overlay.envelope = envelope;
                overlay.guard = guard;
                overlay.limiter = limiter;
                overlay.dynamics = dynamics;
            }

            _initialized = true;
            _lastTimestamp = Time.time;
            _lastTCPPosition = GetCurrentTCPPosition();
            _lastTCPRotation = GetCurrentTCPRotation();

            OnSafetySystemInitialized?.Invoke(this);
        }

        private void AttachGuardCallbacks()
        {
            if (guard != null)
            {
                guard.OnGuardStateChanged += (s) =>
                {
                    if (s.level >= GuardAlertLevel.Critical)
                    {
                        safetyInterventions++;
                        cumulativeEnergySaved += s.tcpKineticEnergy * (1.0 - s.speedRatio);
                        OnSafetyIntervention?.Invoke(safetyInterventions, s.tcpKineticEnergy);
                    }
                    pathsIntercepted = s.interceptedPathCount;
                };
                guard.OnPathIntercepted += (ev) =>
                {
                    pathsIntercepted = guard != null ? guard.totalInterceptions : 0;
                };
            }
        }

        public Vector3d GetCurrentTCPPosition()
        {
            if (useDynamicsForTCP && dynamics != null)
            {
                Quaterniond q = dynamics.bodyState.rotation;
                return dynamics.bodyState.position + q * tcpOffsetFromBody;
            }
            if (cuttingHead != null)
                return cuttingHead.SmoothedPosition;
            return Vector3d.Zero;
        }

        public Quaterniond GetCurrentTCPRotation()
        {
            if (useDynamicsForTCP && dynamics != null)
                return dynamics.bodyState.rotation;
            if (cuttingHead != null)
                return cuttingHead.CurrentRotation;
            return Quaterniond.Identity;
        }

        public Vector3d GetCurrentTCPVelocity()
        {
            if (useDynamicsForTCP && dynamics != null)
                return dynamics.bodyState.linearVelocity +
                       Vector3d.Cross(dynamics.bodyState.angularVelocity,
                                      dynamics.bodyState.rotation * tcpOffsetFromBody);

            double dt = Mathd.Max(1e-4, Time.time - _lastTimestamp);
            Vector3d pos = GetCurrentTCPPosition();
            Vector3d v = (pos - _lastTCPPosition) / dt;
            return v;
        }

        public Vector3d GetCurrentTCPAngularVelocity()
        {
            if (useDynamicsForTCP && dynamics != null)
                return dynamics.bodyState.angularVelocity;

            double dt = Mathd.Max(1e-4, Time.time - _lastTimestamp);
            Quaterniond rot = GetCurrentTCPRotation();
            Quaterniond dq = rot * Quaterniond.Inverse(_lastTCPRotation);
            dq = dq.Normalized;
            double angle = 2 * Mathd.Acos(Mathd.Clamp(dq.w, -1, 1));
            Vector3d axis = new Vector3d(dq.x, dq.y, dq.z);
            double m = axis.Magnitude;
            if (m < 1e-12) return Vector3d.Zero;
            return axis.Normalized * angle / dt;
        }

        private void Update()
        {
            if (!useFixedUpdate && enableSafetySystem)
                RunSafetyCycle(Time.deltaTime);
        }

        private void FixedUpdate()
        {
            if (useFixedUpdate && enableSafetySystem)
            {
                _subStepCounter++;
                if (_subStepCounter >= subStepFactor)
                {
                    RunSafetyCycle(Time.fixedDeltaTime * _subStepCounter);
                    _subStepCounter = 0;
                }
            }
        }

        private void RunSafetyCycle(double deltaTime)
        {
            double t0 = (double)(DateTime.Now.Ticks / 10.0);
            double dt = Mathd.Max(0.0001, deltaTime);

            Vector3d tcpPos = GetCurrentTCPPosition();
            Quaterniond tcpRot = GetCurrentTCPRotation();
            Vector3d tcpLinVel = GetCurrentTCPVelocity();
            Vector3d tcpAngVel = GetCurrentTCPAngularVelocity();

            double spd = tcpLinVel.Magnitude;
            double mass = envelope != null ? envelope.headEquivalentMass : 2500.0;
            double inertia = envelope != null ? envelope.headMomentOfInertia : 120.0;
            double ke = 0.5 * mass * spd * spd + 0.5 * inertia * tcpAngVel.SqrMagnitude;

            debugTCPVelocity = tcpLinVel;
            debugTCPAngularVelocity = tcpAngVel;
            debugTCPSpeed = spd;
            debugTcpKinetic = ke;

            OnTCPUpdated?.Invoke(tcpPos, tcpLinVel, ke);

            if (guard != null)
            {
                GuardState s = guard.UpdateGuard(tcpPos, tcpRot, tcpLinVel, tcpAngVel, dt);
                debugGuardLevel = s.level;
                debugSpeedRatio = s.speedRatio;
                debugEmergency = s.level >= GuardAlertLevel.EmergencyStop;
            }

            if (limiter != null && arm != null)
            {
                limiter.ApplyToArmSkeletonAndHead(dt);
                debugEmergency = limiter.isEmergencyBraking;
            }

            if (overlay != null && overlay.enableOverlay)
            {
                overlay.AddTrajectoryPoint(tcpPos);
                if (guard != null && guard.currentState.level >= GuardAlertLevel.Critical)
                {
                    overlay.ActivateConflictZone(
                        99999 + (int)(Time.time * 100) % 1000,
                        tcpPos,
                        0.8,
                        280e6,
                        $"Guard_{guard.currentState.level}"
                    );
                }
            }

            _lastTCPPosition = tcpPos;
            _lastTCPRotation = tcpRot;
            _lastTimestamp = Time.time;

            double us = (double)(DateTime.Now.Ticks / 10.0) - t0;
            lastFrameExecutionUs = us;
            totalExecutionCalls++;
            averageExecutionUs = averageExecutionUs * (totalExecutionCalls - 1.0) / totalExecutionCalls
                               + us / totalExecutionCalls;
        }

        public void ResetAllSafety()
        {
            if (guard != null) guard.ResetGuard();
            if (limiter != null) limiter.ResetController();
            if (overlay != null)
            {
                overlay.activeZones.Clear();
                overlay.activeConflictCount = 0;
                overlay.trajectoryHistory.Clear();
                overlay.deadlockActive = false;
                overlay.segmentsInConflict = 0;
            }
            safetyInterventions = 0;
            pathsIntercepted = 0;
            cumulativeEnergySaved = 0;
            debugGuardLevel = GuardAlertLevel.Normal;
            debugEmergency = false;
        }

        private void OnGUI()
        {
            int w = 300, h = 130;
            GUILayout.BeginArea(new Rect(Screen.width - w - 15, 410, w, h),
                "安全引擎·执行诊断", "window");

            GUILayout.Label($"执行耗时: 本帧 {lastFrameExecutionUs:F0}μs / 平均 {averageExecutionUs:F1}μs");
            GUILayout.Label($"TCP动能: {debugTcpKinetic / 1000.0:F2} kJ, 速度: {debugTCPSpeed * 100:F1} cm/s");
            GUILayout.Label($"防护状态: {debugGuardLevel}");
            GUILayout.Label($"限速系数: {debugSpeedRatio:F3}x / 急停: {(debugEmergency ? "● 生效" : "○ 未触发")}");
            GUILayout.Label($"总干预/拦截: {safetyInterventions}次 / {pathsIntercepted}次");
            GUILayout.Label($"累计省能: {cumulativeEnergySaved / 1e6:F4} MJ");

            GUILayout.EndArea();
        }
    }
}
