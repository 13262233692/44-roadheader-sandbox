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
    public struct HydraulicAxisState
    {
        public string name;
        public int axisIndex;
        public double currentPosition;
        public double currentVelocity;
        public double targetPosition;
        public double targetVelocity;
        public double limitedVelocity;
        public double currentPressure;
        public double commandedTorque;
        public double appliedTorque;
        public double pidError;
        public double pidIntegral;
        public double pidDerivative;
        public double maxTorque;
        public bool isBraking;
        public bool limitHit;
    }

    [Serializable]
    public class HydraulicSpeedLimiter : MonoBehaviour
    {
        [Header("系统引用")]
        public ArmSkeletonController arm;
        public CuttingHeadMotionController cuttingHead;
        public RoadheaderDynamics dynamics;
        public KineticFeedforwardGuard guard;
        public DHParameterIK ikSolver;

        [Header("六轴PID参数")]
        public double kp = 800.0;
        public double ki = 2.0;
        public double kd = 20.0;
        public double integralDecay = 0.95;
        public double integralLimit = 50000.0;
        public double derivativeFilter = 0.7;

        [Header("液压物理约束")]
        [Tooltip("每轴最大输出扭矩 (N·m)")]
        public double[] perAxisMaxTorque = new double[] { 200000, 250000, 250000, 800000, 50000, 20000 };

        [Tooltip("每轴最大允许角速度 (rad/s)")]
        public double[] perAxisMaxVelocity = new double[] { 0.5, 0.4, 0.6, 0.8, 2.5, 8.0 };

        [Tooltip("每轴角加速度极限 (rad/s²)")]
        public double[] perAxisMaxAcceleration = new double[] { 2.0, 1.5, 2.5, 3.0, 10.0, 30.0 };

        [Header("闭环负反馈控制")]
        [Tooltip("限速控制启用开关")]
        public bool enableLimiter = true;

        [Tooltip("限速前馈系数 (越高跟随越快)")]
        public double feedforwardGain = 0.85;

        [Tooltip("整体速度上限倍率 (安全因子)")]
        public double globalSafetyFactor = 0.85;

        [Tooltip("紧急制动减速度倍率")]
        public double emergencyBrakingGain = 5.0;

        [Header("液压伺服仿真")]
        [Tooltip("液压压力响应时间常数 (s)")]
        public double pressureTimeConstant = 0.02;

        [Tooltip("液压伺服阀开度范围 [0,1]")]
        public double[] currentValveOpening;

        [Header("诊断")]
        public HydraulicAxisState[] axisStates;
        public int controlledAxisCount;
        public double totalCommandedTorque;
        public double totalAppliedTorque;
        public double effectiveSpeedRatio = 1.0;
        public int limitHitsThisFrame;
        public bool isEmergencyBraking;

        public event Action<HydraulicAxisState[]> OnAxisStateUpdated;
        public event Action<int, double, double> OnAxisTorqueApplied;
        public event Action<bool> OnEmergencyBrakingChanged;

        private double[] _previousError;
        private double[] _integralAccum;
        private double[] _filteredDerivative;
        private double _lastDeltaTime;

        private void Awake()
        {
            controlledAxisCount = 6;
            axisStates = new HydraulicAxisState[controlledAxisCount];
            _previousError = new double[controlledAxisCount];
            _integralAccum = new double[controlledAxisCount];
            _filteredDerivative = new double[controlledAxisCount];
            currentValveOpening = new double[controlledAxisCount];

            string[] axisNames = new string[]
            {
                "回转台Yaw", "大臂Pitch", "小臂Pitch",
                "伸缩缸", "截割头Roll", "截割头Spin"
            };

            for (int i = 0; i < controlledAxisCount; i++)
            {
                axisStates[i] = new HydraulicAxisState
                {
                    name = axisNames[i],
                    axisIndex = i,
                    maxTorque = i < perAxisMaxTorque.Length ? perAxisMaxTorque[i] : 100000
                };
                currentValveOpening[i] = 0;
            }
        }

        public void InitializeReferences(ArmSkeletonController a, CuttingHeadMotionController ch,
                                          RoadheaderDynamics d, KineticFeedforwardGuard g, DHParameterIK ik)
        {
            arm = a;
            cuttingHead = ch;
            dynamics = d;
            guard = g;
            ikSolver = ik;
        }

        public double[] StepLimiter(double[] targetVelocities, double[] currentPositions, double deltaTime)
        {
            double dt = Mathd.Max(0.0001, deltaTime);
            _lastDeltaTime = dt;

            double ratio = guard != null ? guard.currentSpeedRatio : 1.0;
            effectiveSpeedRatio = ratio * globalSafetyFactor;

            isEmergencyBraking = guard != null &&
                (guard.displayLevel == GuardAlertLevel.EmergencyStop ||
                 guard.displayLevel == GuardAlertLevel.Intercepted);

            double[] limited = new double[controlledAxisCount];
            totalCommandedTorque = 0;
            totalAppliedTorque = 0;
            limitHitsThisFrame = 0;

            for (int i = 0; i < controlledAxisCount; i++)
            {
                double tgtVel = targetVelocities[i];
                double maxV = i < perAxisMaxVelocity.Length ? perAxisMaxVelocity[i] : 1.0;
                double maxA = i < perAxisMaxAcceleration.Length ? perAxisMaxAcceleration[i] : 10.0;
                double curV = axisStates[i].currentVelocity;

                double scaledTargetVel = tgtVel * effectiveSpeedRatio;

                if (isEmergencyBraking)
                {
                    double decelTgt = -curV * emergencyBrakingGain * 0.1;
                    scaledTargetVel = Mathd.MoveTowards(scaledTargetVel, decelTgt, Mathd.Abs(curV) * 0.3);
                }

                double maxDelta = maxA * dt;
                limited[i] = Mathd.MoveTowards(curV, scaledTargetVel, maxDelta);
                limited[i] = Mathd.Clamp(limited[i], -maxV, maxV);

                if (Mathd.Abs(limited[i] - scaledTargetVel) > 1e-6 ||
                    Mathd.Abs(limited[i]) >= maxV * 0.995)
                {
                    axisStates[i].limitHit = true;
                    limitHitsThisFrame++;
                }
                else axisStates[i].limitHit = false;

                double pidTorque = ComputePID(i, targetVelocities[i], limited[i], currentPositions[i], dt);
                double feedforwardTorque = (limited[i] - curV) / Mathd.Max(0.0001, dt) *
                    (i < perAxisMaxTorque.Length ? perAxisMaxTorque[i] * 0.02 : 2000);
                double totalT = pidTorque + feedforwardGain * feedforwardTorque;

                double maxT = axisStates[i].maxTorque;
                if (isEmergencyBraking) maxT *= emergencyBrakingGain;
                totalT = Mathd.Clamp(totalT, -maxT, maxT);

                double pTgt = Mathd.Abs(totalT) / Mathd.Max(1.0, axisStates[i].maxTorque);
                currentValveOpening[i] = Mathd.Lerp(currentValveOpening[i], pTgt,
                    dt / Mathd.Max(0.001, pressureTimeConstant));

                axisStates[i].targetPosition = currentPositions[i] + targetVelocities[i] * dt;
                axisStates[i].targetVelocity = targetVelocities[i];
                axisStates[i].limitedVelocity = limited[i];
                axisStates[i].currentPosition = currentPositions[i];
                axisStates[i].currentVelocity = limited[i];
                axisStates[i].commandedTorque = pidTorque;
                axisStates[i].appliedTorque = totalT;
                axisStates[i].currentPressure = currentValveOpening[i] * 30e6;
                axisStates[i].isBraking = (targetVelocities[i] * curV < 0) || isEmergencyBraking;

                totalCommandedTorque += Mathd.Abs(pidTorque);
                totalAppliedTorque += Mathd.Abs(totalT);

                OnAxisTorqueApplied?.Invoke(i, totalT, limited[i]);
            }

            OnAxisStateUpdated?.Invoke(axisStates);
            return limited;
        }

        private double ComputePID(int idx, double tgtVel, double actVel, double curPos, double dt)
        {
            double error = tgtVel - actVel;
            if (double.IsNaN(error) || double.IsInfinity(error)) error = 0;

            _integralAccum[idx] = _integralAccum[idx] * integralDecay + error * dt;
            _integralAccum[idx] = Mathd.Clamp(_integralAccum[idx], -integralLimit, integralLimit);

            double rawDeriv = (error - _previousError[idx]) / Mathd.Max(1e-6, dt);
            _filteredDerivative[idx] = _filteredDerivative[idx] * derivativeFilter +
                                        rawDeriv * (1.0 - derivativeFilter);
            if (double.IsNaN(_filteredDerivative[idx]) || double.IsInfinity(_filteredDerivative[idx]))
                _filteredDerivative[idx] = 0;

            _previousError[idx] = error;

            double outT = kp * error + ki * _integralAccum[idx] + kd * _filteredDerivative[idx];
            axisStates[idx].pidError = error;
            axisStates[idx].pidIntegral = _integralAccum[idx];
            axisStates[idx].pidDerivative = _filteredDerivative[idx];
            return outT;
        }

        public void ApplyToArmSkeletonAndHead(double deltaTime)
        {
            double dt = Mathd.Max(0.0001, deltaTime);
            double[] positions = new double[controlledAxisCount];
            double[] velocities = new double[controlledAxisCount];

            for (int i = 0; i < controlledAxisCount && arm != null && i < arm.segments.Count; i++)
            {
                var seg = arm.segments[i];
                positions[i] = seg.currentAngle;
                velocities[i] = seg.angularVelocity;
            }

            if (cuttingHead != null)
            {
                int spinIdx = 5;
                if (spinIdx < controlledAxisCount)
                {
                    positions[spinIdx] = cuttingHead.CurrentSpinAngle;
                    velocities[spinIdx] = cuttingHead.spinSpeed;
                }
                int rollIdx = 4;
                if (rollIdx < controlledAxisCount)
                {
                    positions[rollIdx] = 0;
                    velocities[rollIdx] = 0;
                }
            }

            double[] limitedVels = StepLimiter(velocities, positions, dt);

            if (arm != null)
            {
                for (int i = 0; i < arm.segments.Count && i < controlledAxisCount; i++)
                {
                    var seg = arm.segments[i];
                    seg.angularVelocity = Mathd.Clamp(limitedVels[i],
                        -seg.maxAngularVelocity * Mathd.Deg2Rad,
                         seg.maxAngularVelocity * Mathd.Deg2Rad);
                    seg.targetAngle = seg.currentAngle + seg.angularVelocity * dt;
                    seg.UpdateDynamics(dt);
                }
            }

            if (cuttingHead != null)
            {
                int spinIdx = 5;
                if (spinIdx < controlledAxisCount && limitedVels[spinIdx] != 0)
                {
                    double ratio = Mathd.Abs(cuttingHead.spinSpeed) > 1e-6
                        ? Mathd.Clamp(Mathd.Abs(limitedVels[spinIdx]) / Mathd.Max(1e-6, Mathd.Abs(cuttingHead.spinSpeed)), 0, 1)
                        : 0;
                    cuttingHead.spinSpeed *= ratio;
                }

                int traverseIdx = 2;
                if (traverseIdx < controlledAxisCount)
                {
                    double tRatio = Mathd.Abs(velocities[traverseIdx]) > 1e-6
                        ? Mathd.Clamp(Mathd.Abs(limitedVels[traverseIdx]) / Mathd.Abs(velocities[traverseIdx]), 0.05, 1)
                        : effectiveSpeedRatio;
                    cuttingHead.traverseSpeed = Mathd.Sign(cuttingHead.traverseSpeed) *
                        Mathd.Abs(cuttingHead.traverseSpeed) * tRatio;
                }
            }
        }

        public void ResetController()
        {
            for (int i = 0; i < controlledAxisCount; i++)
            {
                _previousError[i] = 0;
                _integralAccum[i] = 0;
                _filteredDerivative[i] = 0;
                currentValveOpening[i] = 0;
                axisStates[i].currentVelocity = 0;
                axisStates[i].pidError = 0;
                axisStates[i].pidIntegral = 0;
                axisStates[i].pidDerivative = 0;
                axisStates[i].commandedTorque = 0;
                axisStates[i].appliedTorque = 0;
                axisStates[i].limitHit = false;
                axisStates[i].isBraking = false;
            }
            effectiveSpeedRatio = 1.0;
            isEmergencyBraking = false;
            OnEmergencyBrakingChanged?.Invoke(false);
        }
    }
}
