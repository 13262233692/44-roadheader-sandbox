using System;
using System.Collections.Generic;
using RoadheaderSandbox.Core.Math;
using UnityEngine;

namespace RoadheaderSandbox.Kinematics
{
    [Serializable]
    public class ArmSegment
    {
        public string name;
        public Transform transform;
        public double length = 1.0;
        public Vector3d jointAxis = Vector3d.Up;
        public double minAngle = -90.0;
        public double maxAngle = 90.0;
        public double currentAngle;
        public double targetAngle;
        public double angularVelocity;
        public double maxAngularVelocity = 180.0;

        [Tooltip("单步最大角度变化 (rad)，防止鬼抽")]
        public double maxAngleDeltaPerStep = 5.0 * Mathd.Deg2Rad;

        public double damping = 5.0;
        public double stiffness = 100.0;

        private Quaterniond _initialRotation;
        private Quaterniond _currentRotation;
        private bool _rotationInitialized;

        public Vector3d StartPosition => transform != null ? Vector3d.FromVector3(transform.position) : Vector3d.Zero;

        public Vector3d EndPosition
        {
            get
            {
                if (transform == null) return Vector3d.Zero;
                Vector3d forward = Quaterniond.FromQuaternion(transform.rotation) * Vector3d.Forward;
                return StartPosition + forward * length;
            }
        }

        public void InitializeRotation()
        {
            if (transform == null || _rotationInitialized) return;
            _initialRotation = Quaterniond.FromQuaternion(transform.localRotation);
            _currentRotation = _initialRotation;
            _rotationInitialized = true;
        }

        public void ApplyAngle(double angle, double deltaTime)
        {
            if (deltaTime <= 0) deltaTime = 0.001;
            InitializeRotation();

            targetAngle = Mathd.Clamp(angle, minAngle, maxAngle);
            double maxDelta = maxAngularVelocity * Mathd.Deg2Rad * deltaTime;
            maxDelta = Mathd.Min(maxDelta, maxAngleDeltaPerStep);
            double newAngle = Mathd.MoveTowardsAngle(currentAngle, targetAngle, maxDelta);
            currentAngle = newAngle;

            if (transform != null && jointAxis.SqrMagnitude > 1e-15)
            {
                Vector3d axis = jointAxis.Normalized;
                Quaterniond axisRot = Quaterniond.AxisAngle(axis, currentAngle);
                Quaterniond targetRot = (axisRot * _initialRotation).Normalized;
                double slerpT = Mathd.Clamp01(deltaTime * 50.0);
                _currentRotation = Quaterniond.Slerp(_currentRotation, targetRot, slerpT);
                transform.localRotation = _currentRotation.ToQuaternion();
            }
        }

        public void UpdateDynamics(double deltaTime)
        {
            if (deltaTime <= 0) deltaTime = 0.001;
            InitializeRotation();

            double angleError = targetAngle - currentAngle;
            angularVelocity += angleError * stiffness * deltaTime;
            angularVelocity *= Mathd.Max(0, 1.0 - damping * deltaTime);
            double maxOmega = maxAngularVelocity * Mathd.Deg2Rad;
            angularVelocity = Mathd.Clamp(angularVelocity, -maxOmega, maxOmega);

            double maxDelta = maxOmega * deltaTime;
            maxDelta = Mathd.Min(maxDelta, maxAngleDeltaPerStep);
            double angleDelta = Mathd.Clamp(angularVelocity * deltaTime, -maxDelta, maxDelta);
            currentAngle += angleDelta;
            currentAngle = Mathd.Clamp(currentAngle, minAngle, maxAngle);

            if (transform != null && jointAxis.SqrMagnitude > 1e-15)
            {
                Vector3d axis = jointAxis.Normalized;
                Quaterniond axisRot = Quaterniond.AxisAngle(axis, currentAngle);
                Quaterniond targetRot = (axisRot * _initialRotation).Normalized;
                double slerpT = Mathd.Clamp01(deltaTime * 50.0);
                _currentRotation = Quaterniond.Slerp(_currentRotation, targetRot, slerpT);
                transform.localRotation = _currentRotation.ToQuaternion();
            }
        }

        public void ResetToInitial()
        {
            InitializeRotation();
            currentAngle = 0;
            targetAngle = 0;
            angularVelocity = 0;
            if (transform != null)
            {
                _currentRotation = _initialRotation;
                transform.localRotation = _initialRotation.ToQuaternion();
            }
        }
    }

    [Serializable]
    public class ArmSkeletonController : MonoBehaviour
    {
        [Header("臂节配置")]
        public List<ArmSegment> segments = new List<ArmSegment>();

        [Header("末端执行器")]
        public Transform endEffector;
        public CuttingHeadMotionController cuttingHeadController;

        [Header("逆运动学")]
        public bool useInverseKinematics = true;
        public int ikIterations = 50;
        public double ikTolerance = 0.001;
        public double ikDamping = 0.1;

        [Tooltip("IK单步最大角度变化 (rad)")]
        public double ikMaxAngleDelta = 2.0 * Mathd.Deg2Rad;

        [Header("机体坐标系")]
        public Transform bodyFrame;
        public bool useBodyFrame = true;

        [Header("动力学")]
        public bool enableDynamics = true;
        public double gravity = 9.81;

        [Header("数值稳定器")]
        [Tooltip("整体位置硬约束半径 (m)")]
        public double maxReachHardLimit = 6.0;

        private Vector3d _targetPosition;
        private Quaterniond _targetRotation;
        private bool _hasTarget;
        private Vector3d _lastValidEndPosition;

        public event Action<int, double> OnSegmentAngleChanged;
        public event Action<Vector3d> OnEndEffectorPositionChanged;

        public int SegmentCount => segments.Count;
        public Vector3d EndEffectorPosition => GetEndEffectorPosition();
        public Vector3d TargetPosition => _targetPosition;
        public bool HasTarget => _hasTarget;

        private void Awake()
        {
            InitializeSegments();
        }

        public void InitializeSegments()
        {
            for (int i = 0; i < segments.Count; i++)
            {
                if (segments[i].transform != null)
                {
                    segments[i].InitializeRotation();
                    segments[i].currentAngle = 0;
                    segments[i].targetAngle = 0;
                    segments[i].angularVelocity = 0;
                }
            }
            _lastValidEndPosition = GetEndEffectorPosition();
        }

        public void SetTarget(Vector3d position, Quaterniond rotation)
        {
            _targetPosition = position;
            _targetRotation = rotation;
            _hasTarget = true;
            ClampTargetPosition();
        }

        public void SetTargetPosition(Vector3d position)
        {
            _targetPosition = position;
            _hasTarget = true;
            ClampTargetPosition();
        }

        private void ClampTargetPosition()
        {
            if (segments.Count == 0) return;

            Vector3d basePos = segments[0].StartPosition;
            Vector3d toTarget = _targetPosition - basePos;
            double distance = toTarget.Magnitude;

            if (distance > maxReachHardLimit)
            {
                _targetPosition = basePos + toTarget.Normalized * maxReachHardLimit;
            }

            if (double.IsNaN(_targetPosition.x) || double.IsNaN(_targetPosition.y) || double.IsNaN(_targetPosition.z) ||
                double.IsInfinity(_targetPosition.x) || double.IsInfinity(_targetPosition.y) || double.IsInfinity(_targetPosition.z))
            {
                _targetPosition = _lastValidEndPosition;
            }
        }

        public void ClearTarget()
        {
            _hasTarget = false;
        }

        public Vector3d GetEndEffectorPosition()
        {
            if (endEffector != null)
                return Vector3d.FromVector3(endEffector.position);

            if (segments.Count > 0)
                return segments[segments.Count - 1].EndPosition;

            return Vector3d.FromVector3(transform.position);
        }

        public Vector3d GetSegmentGlobalPosition(int segmentIndex)
        {
            if (segmentIndex < 0 || segmentIndex >= segments.Count)
                return Vector3d.Zero;
            return segments[segmentIndex].StartPosition;
        }

        public double GetSegmentAngle(int segmentIndex)
        {
            if (segmentIndex < 0 || segmentIndex >= segments.Count)
                return 0;
            return segments[segmentIndex].currentAngle;
        }

        public void SetSegmentAngle(int segmentIndex, double angle)
        {
            if (segmentIndex < 0 || segmentIndex >= segments.Count) return;
            segments[segmentIndex].ApplyAngle(angle, Time.fixedDeltaTime);
            OnSegmentAngleChanged?.Invoke(segmentIndex, segments[segmentIndex].currentAngle);
        }

        public void UpdateKinematics(double deltaTime)
        {
            if (deltaTime <= 0) deltaTime = 0.001;

            if (_hasTarget && useInverseKinematics)
            {
                SolveInverseKinematics();
            }

            for (int i = 0; i < segments.Count; i++)
            {
                if (enableDynamics)
                {
                    segments[i].UpdateDynamics(deltaTime);
                }
                else
                {
                    segments[i].ApplyAngle(segments[i].targetAngle, deltaTime);
                }
                OnSegmentAngleChanged?.Invoke(i, segments[i].currentAngle);
            }

            Vector3d endPos = GetEndEffectorPosition();
            if (!double.IsNaN(endPos.x) && !double.IsNaN(endPos.y) && !double.IsNaN(endPos.z) &&
                !double.IsInfinity(endPos.x) && !double.IsInfinity(endPos.y) && !double.IsInfinity(endPos.z))
            {
                _lastValidEndPosition = endPos;
            }
            OnEndEffectorPositionChanged?.Invoke(_lastValidEndPosition);
        }

        private void SolveInverseKinematics()
        {
            if (segments.Count == 0) return;

            Vector3d target = _targetPosition;
            if (useBodyFrame && bodyFrame != null)
            {
                Matrix4x4d bodyToWorld = Matrix4x4d.TRS(
                    Vector3d.FromVector3(bodyFrame.position),
                    Quaterniond.FromQuaternion(bodyFrame.rotation),
                    Vector3d.One
                );
                target = bodyToWorld.Inverse.MultiplyPoint(target);
            }

            double[] angles = new double[segments.Count];
            for (int i = 0; i < segments.Count; i++)
            {
                angles[i] = segments[i].currentAngle;
            }

            for (int iteration = 0; iteration < ikIterations; iteration++)
            {
                Vector3d endPos = ForwardKinematics(angles);
                Vector3d error = target - endPos;

                if (error.SqrMagnitude < ikTolerance * ikTolerance)
                    break;

                double[,] jacobian = CalculateJacobian(angles);
                double[] deltaAngles = SolveJacobianTranspose(jacobian, error, angles);

                for (int i = 0; i < segments.Count; i++)
                {
                    double clampedDelta = Mathd.Clamp(deltaAngles[i], -ikMaxAngleDelta, ikMaxAngleDelta);
                    angles[i] += clampedDelta;
                    angles[i] = Mathd.Clamp(angles[i], segments[i].minAngle, segments[i].maxAngle);
                }
            }

            for (int i = 0; i < segments.Count; i++)
            {
                double current = segments[i].targetAngle;
                double next = angles[i];
                double maxDelta = segments[i].maxAngleDeltaPerStep;
                segments[i].targetAngle = Mathd.MoveTowardsAngle(current, next, maxDelta);
            }
        }

        private Vector3d ForwardKinematics(double[] angles)
        {
            Vector3d position = Vector3d.Zero;
            Quaterniond rotation = Quaterniond.Identity;

            for (int i = 0; i < segments.Count; i++)
            {
                rotation = rotation * Quaterniond.AxisAngle(segments[i].jointAxis, angles[i]);
                Vector3d segmentEnd = rotation * (Vector3d.Forward * segments[i].length);
                position += segmentEnd;
            }

            return position;
        }

        private double[,] CalculateJacobian(double[] angles)
        {
            int n = segments.Count;
            double[,] jacobian = new double[3, n];

            Vector3d[] jointPositions = new Vector3d[n + 1];
            Quaterniond[] jointRotations = new Quaterniond[n + 1];

            jointPositions[0] = Vector3d.Zero;
            jointRotations[0] = Quaterniond.Identity;

            for (int i = 0; i < n; i++)
            {
                jointRotations[i + 1] = jointRotations[i] * Quaterniond.AxisAngle(segments[i].jointAxis, angles[i]);
                Vector3d segmentEnd = jointRotations[i + 1] * (Vector3d.Forward * segments[i].length);
                jointPositions[i + 1] = jointPositions[i] + segmentEnd;
            }

            Vector3d endPos = jointPositions[n];

            for (int i = 0; i < n; i++)
            {
                Vector3d axis = jointRotations[i] * segments[i].jointAxis;
                if (axis.SqrMagnitude < 1e-15) continue;
                axis = axis.Normalized;

                Vector3d lever = endPos - jointPositions[i];
                Vector3d velocity = Vector3d.Cross(axis, lever);

                jacobian[0, i] = velocity.x;
                jacobian[1, i] = velocity.y;
                jacobian[2, i] = velocity.z;
            }

            return jacobian;
        }

        private double[] SolveJacobianTranspose(double[,] jacobian, Vector3d error, double[] angles)
        {
            int n = angles.Length;
            double[] deltaAngles = new double[n];

            for (int i = 0; i < n; i++)
            {
                deltaAngles[i] = jacobian[0, i] * error.x + jacobian[1, i] * error.y + jacobian[2, i] * error.z;
                deltaAngles[i] *= ikDamping;
            }

            return deltaAngles;
        }

        public double GetTotalReach()
        {
            double total = 0;
            foreach (var segment in segments)
            {
                total += segment.length;
            }
            return total;
        }

        public bool IsReachable(Vector3d target)
        {
            double distance = Vector3d.Distance(Vector3d.Zero, target);
            return distance <= GetTotalReach();
        }

        private void FixedUpdate()
        {
            if (cuttingHeadController != null)
            {
                SetTargetPosition(cuttingHeadController.SmoothedPosition);
            }
            UpdateKinematics(Time.fixedDeltaTime);
        }

        private void OnDrawGizmosSelected()
        {
            if (segments.Count == 0) return;

            Vector3d prevPos = segments[0].StartPosition;

            for (int i = 0; i < segments.Count; i++)
            {
                Vector3d endPos = segments[i].EndPosition;

                Gizmos.color = Color.Lerp(Color.blue, Color.green, (float)i / Mathf.Max(1, segments.Count - 1));
                Gizmos.DrawLine(prevPos.ToVector3(), endPos.ToVector3());
                Gizmos.DrawSphere(endPos.ToVector3(), 0.05f);

                if (segments[i].transform != null)
                {
                    Gizmos.color = Color.yellow;
                    Vector3 axis = segments[i].jointAxis.ToVector3();
                    Gizmos.DrawRay(segments[i].transform.position, axis * 0.3f);
                }

                prevPos = endPos;
            }

            if (_hasTarget)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(_targetPosition.ToVector3(), 0.1f);

                Gizmos.color = new Color(1, 0, 0, 0.1f);
                if (segments.Count > 0)
                {
                    Gizmos.DrawWireSphere(segments[0].StartPosition.ToVector3(), (float)maxReachHardLimit);
                }
            }
        }
    }
}
