using System;
using System.Collections.Generic;
using RoadheaderSandbox.Core.Math;
using RoadheaderSandbox.Physics;
using UnityEngine;

namespace RoadheaderSandbox.Safety
{
    [Serializable]
    public enum GuardAlertLevel
    {
        Normal = 0,
        Warning = 1,
        Critical = 2,
        EmergencyStop = 3,
        Intercepted = 4
    }

    [Serializable]
    public struct GuardState
    {
        public GuardAlertLevel level;
        public double timestamp;
        public Vector3d tcpPosition;
        public double tcpKineticEnergy;
        public double speedRatio;
        public double jointBrakingTorque;
        public int interceptedPathCount;
        public string message;

        public static GuardState Safe => new GuardState
        {
            level = GuardAlertLevel.Normal,
            speedRatio = 1.0,
            message = "正常运行"
        };
    }

    [Serializable]
    public struct PathInterceptionEvent
    {
        public double time;
        public Vector3d dangerPoint;
        public Vector3d safePoint;
        public double originalSpeed;
        public double interceptedSpeed;
        public double overloadRatio;
        public int boundaryId;
        public string reason;
    }

    [Serializable]
    public class KineticFeedforwardGuard : MonoBehaviour
    {
        [Header("核心引擎引用")]
        public TCPSpaceEnvelope envelope;
        public DHParameterIK ikSolver;
        public CuttingHeadCollisionSolver collisionSolver;
        public RockSurface rockSurface;

        [Header("防护等级阈值")]
        [Tooltip("动能预警阈值 (J) - 黄色警告")]
        public double warningEnergyThreshold = 20000.0;

        [Tooltip("动能临界阈值 (J) - 红色紧急 - 触发限速")]
        public double criticalEnergyThreshold = 40000.0;

        [Tooltip("硬度预警阈值 (Pa)")]
        public double warningHardnessThreshold = 120e6;

        [Tooltip("硬度临界阈值 (Pa) - 触发拦截")]
        public double criticalHardnessThreshold = 200e6;

        [Tooltip("力过载预警阈值系数")]
        public double warningForceRatio = 0.6;

        [Tooltip("力过载截停阈值系数")]
        public double emergencyForceRatio = 0.95;

        [Header("限速控制")]
        [Tooltip("预警等级限速系数")]
        public double warningSpeedRatio = 0.6;

        [Tooltip("临界等级限速系数")]
        public double criticalSpeedRatio = 0.2;

        [Tooltip("急停等级限速系数")]
        public double emergencyStopSpeedRatio = 0.0;

        [Tooltip("限速恢复平滑系数 (越大越快)")]
        public double recoverySmoothFactor = 0.3;

        [Header("微秒级硬拦截")]
        [Tooltip("启用预判路径强行拦截")]
        public bool enableHardInterception = true;

        [Tooltip("拦截生效提前量 (s)")]
        public double interceptionLookahead = 0.15;

        [Tooltip("单步最大允许速度变化率")]
        public double maxDecelerationRate = 50.0;

        [Header("诊断记录")]
        public GuardState currentState;
        public GuardAlertLevel displayLevel;
        public double currentSpeedRatio = 1.0;
        public double currentTCPKineticEnergy;
        public double currentHardnessOnPath;
        public int totalInterceptions;
        public List<PathInterceptionEvent> interceptionHistory = new List<PathInterceptionEvent>(100);

        public event Action<GuardState> OnGuardStateChanged;
        public event Action<PathInterceptionEvent> OnPathIntercepted;
        public event Action<string, GuardAlertLevel> OnAlertRaised;

        private double _lastTime;
        private double _smoothedRatio = 1.0;
        private GuardAlertLevel _lastLevel;

        private void Awake()
        {
            if (envelope == null)
                envelope = FindObjectOfType<TCPSpaceEnvelope>();
            if (ikSolver == null)
                ikSolver = FindObjectOfType<DHParameterIK>();
            if (collisionSolver == null)
                collisionSolver = FindObjectOfType<CuttingHeadCollisionSolver>();
            if (rockSurface == null)
                rockSurface = FindObjectOfType<RockSurface>();

            currentState = GuardState.Safe;
            _lastLevel = GuardAlertLevel.Normal;
            _smoothedRatio = 1.0;
            _lastTime = Time.time;
        }

        public GuardState UpdateGuard(Vector3d tcpPosition, Quaterniond tcpRotation,
                                       Vector3d linearVel, Vector3d angularVel, double deltaTime)
        {
            double dt = Mathd.Max(0.0001, deltaTime);
            double t = Time.time;

            if (envelope != null)
            {
                CollisionPrediction prediction = envelope.Compute(tcpPosition, tcpRotation, linearVel, angularVel, dt);
                return EvaluateAndApply(tcpPosition, linearVel, angularVel, prediction, dt, t);
            }

            currentTCPKineticEnergy = CalculateKineticEnergy(linearVel, angularVel);
            currentHardnessOnPath = envelope != null ? envelope.SampleHardness(tcpPosition) : 80e6;
            GuardAlertLevel level = ClassifyRisk(currentTCPKineticEnergy, currentHardnessOnPath, 0);
            return FinalizeState(level, tcpPosition, 1.0, t, "快速路径检查", null);
        }

        private GuardState EvaluateAndApply(Vector3d tcpPos, Vector3d linVel, Vector3d angVel,
                                             CollisionPrediction pred, double dt, double time)
        {
            currentTCPKineticEnergy = envelope != null ? envelope.maxKineticEnergyOnPath
                                                       : CalculateKineticEnergy(linVel, angVel);
            currentHardnessOnPath = envelope != null ? envelope.maxHardnessOnPath : 80e6;

            double collisionForceRatio = pred.willCollide
                ? pred.impactForce / Mathd.Max(1.0, allowablePickForce() * 42)
                : 0;

            GuardAlertLevel level = ClassifyRisk(currentTCPKineticEnergy, currentHardnessOnPath, collisionForceRatio);

            if (pred.willCollide && pred.collisionTime <= interceptionLookahead)
                level = (GuardAlertLevel)Mathd.Max((int)level, (int)GuardAlertLevel.Intercepted);

            if (pred.willCollide && pred.overloadRatio > 1.5)
                level = (GuardAlertLevel)Mathd.Max((int)level, (int)GuardAlertLevel.EmergencyStop);

            double targetRatio = ComputeTargetSpeedRatio(level, pred);
            double maxDelta = maxDecelerationRate * dt;
            _smoothedRatio = Mathd.MoveTowards(_smoothedRatio, targetRatio, maxDelta);
            _smoothedRatio = Mathd.Clamp(_smoothedRatio, emergencyStopSpeedRatio, 1.0);

            if (enableHardInterception && pred.willCollide && pred.collisionTime <= interceptionLookahead + 0.2)
            {
                HardInterceptIfNeeded(tcpPos, linVel, pred, time);
            }

            string reason = BuildAlertReason(level, pred);
            return FinalizeState(level, tcpPos, _smoothedRatio, time, reason, pred.willCollide ? (PathInterceptionEvent?)null : null);
        }

        private double allowablePickForce()
        {
            return collisionSolver != null ? collisionSolver.maxPickContactForce : 30000.0;
        }

        private GuardAlertLevel ClassifyRisk(double ke, double hardness, double forceRatio)
        {
            int score = 0;
            if (ke >= warningEnergyThreshold) score++;
            if (ke >= criticalEnergyThreshold) score++;
            if (hardness >= warningHardnessThreshold) score++;
            if (hardness >= criticalHardnessThreshold) score++;
            if (forceRatio >= warningForceRatio) score++;
            if (forceRatio >= emergencyForceRatio) score += 2;

            if (score >= 5) return GuardAlertLevel.EmergencyStop;
            if (score >= 3) return GuardAlertLevel.Critical;
            if (score >= 1) return GuardAlertLevel.Warning;
            return GuardAlertLevel.Normal;
        }

        private double ComputeTargetSpeedRatio(GuardAlertLevel level, CollisionPrediction pred)
        {
            if (pred.willCollide && pred.requiredSpeedReductionRatio < 1.0)
                return pred.requiredSpeedReductionRatio;

            switch (level)
            {
                case GuardAlertLevel.Warning: return warningSpeedRatio;
                case GuardAlertLevel.Critical: return criticalSpeedRatio;
                case GuardAlertLevel.EmergencyStop:
                case GuardAlertLevel.Intercepted: return emergencyStopSpeedRatio;
                default: return 1.0;
            }
        }

        private string BuildAlertReason(GuardAlertLevel level, CollisionPrediction pred)
        {
            if (level == GuardAlertLevel.Normal) return "正常运行";

            System.Text.StringBuilder sb = new System.Text.StringBuilder();
            if (currentTCPKineticEnergy >= warningEnergyThreshold)
                sb.Append($"[KE:{currentTCPKineticEnergy / 1000.0:F1}kJ]");
            if (currentHardnessOnPath >= warningHardnessThreshold)
                sb.Append($"[H:{currentHardnessOnPath / 1e6:F1}MPa]");
            if (pred.willCollide)
                sb.Append($"[碰撞@{pred.collisionTime * 1000:F0}ms 过载x{pred.overloadRatio:F2}]");
            if (level >= GuardAlertLevel.Critical)
                sb.Append($"[限速x{_smoothedRatio:F2}]");
            return sb.ToString();
        }

        private void HardInterceptIfNeeded(Vector3d tcpPos, Vector3d linVel, CollisionPrediction pred, double time)
        {
            if (!enableHardInterception) return;

            Vector3d danger = pred.collisionPoint;
            Vector3d safeDir = (tcpPos - danger);
            if (safeDir.SqrMagnitude < 1e-12) safeDir = -linVel;
            if (safeDir.SqrMagnitude < 1e-12) safeDir = Vector3d.Up;
            Vector3d safePoint = tcpPos + safeDir.Normalized * 0.3;

            PathInterceptionEvent ev = new PathInterceptionEvent
            {
                time = time,
                dangerPoint = danger,
                safePoint = safePoint,
                originalSpeed = linVel.Magnitude,
                interceptedSpeed = linVel.Magnitude * pred.requiredSpeedReductionRatio,
                overloadRatio = pred.overloadRatio,
                boundaryId = pred.boundaryId,
                reason = pred.targetHardness > 250e6 ? "高硬度岩芯拦截" : "动能过载拦截"
            };

            if (interceptionHistory.Count >= 100)
                interceptionHistory.RemoveAt(0);
            interceptionHistory.Add(ev);
            totalInterceptions++;

            OnPathIntercepted?.Invoke(ev);
        }

        private GuardState FinalizeState(GuardAlertLevel level, Vector3d tcp, double ratio,
                                          double time, string msg, PathInterceptionEvent? ev)
        {
            currentState = new GuardState
            {
                level = level,
                timestamp = time,
                tcpPosition = tcp,
                tcpKineticEnergy = currentTCPKineticEnergy,
                speedRatio = ratio,
                jointBrakingTorque = (1.0 - ratio) * 150000,
                interceptedPathCount = totalInterceptions,
                message = msg
            };

            currentSpeedRatio = ratio;
            displayLevel = level;

            if (level != _lastLevel)
            {
                _lastLevel = level;
                OnGuardStateChanged?.Invoke(currentState);
                if (level >= GuardAlertLevel.Warning)
                    OnAlertRaised?.Invoke(msg, level);
            }

            return currentState;
        }

        private double CalculateKineticEnergy(Vector3d linVel, Vector3d angVel)
        {
            double mass = envelope != null ? envelope.headEquivalentMass : 2500.0;
            double inertia = envelope != null ? envelope.headMomentOfInertia : 120.0;
            return 0.5 * mass * linVel.SqrMagnitude + 0.5 * inertia * angVel.SqrMagnitude;
        }

        public void ResetGuard()
        {
            currentState = GuardState.Safe;
            _smoothedRatio = 1.0;
            currentSpeedRatio = 1.0;
            totalInterceptions = 0;
            interceptionHistory.Clear();
            _lastLevel = GuardAlertLevel.Normal;
            OnGuardStateChanged?.Invoke(currentState);
        }

        private void OnDrawGizmos()
        {
            if (displayLevel <= GuardAlertLevel.Normal) return;

            Color baseColor = Color.white;
            switch (displayLevel)
            {
                case GuardAlertLevel.Warning: baseColor = new Color(1, 0.85f, 0, 0.9f); break;
                case GuardAlertLevel.Critical: baseColor = new Color(1, 0.4f, 0, 0.95f); break;
                case GuardAlertLevel.EmergencyStop:
                case GuardAlertLevel.Intercepted: baseColor = new Color(1, 0.1f, 0.2f, 1f); break;
            }

            float pulse = 0.6f + 0.4f * Mathf.Sin(Time.time * 8.0f);
            Gizmos.color = baseColor * pulse;

            if (currentState.tcpPosition != Vector3d.Zero)
            {
                Gizmos.DrawWireSphere(currentState.tcpPosition.ToVector3(), 0.4f + pulse * 0.1f);
                Gizmos.DrawWireSphere(currentState.tcpPosition.ToVector3(), 0.7f + pulse * 0.2f);
            }

            if (envelope != null && envelope.latestPrediction.willCollide)
            {
                Gizmos.color = new Color(1, 0, 0, 0.8f);
                Gizmos.DrawLine(currentState.tcpPosition.ToVector3(),
                                envelope.latestPrediction.collisionPoint.ToVector3());
                Gizmos.color = new Color(1, 0.2f, 0.2f, 0.6f) * pulse;
                Gizmos.DrawSphere(envelope.latestPrediction.collisionPoint.ToVector3(), 0.25f);
            }
        }
    }
}
