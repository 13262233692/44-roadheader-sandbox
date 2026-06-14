using System;
using System.Collections.Generic;
using RoadheaderSandbox.Core.Math;
using UnityEngine;

namespace RoadheaderSandbox.Kinematics
{
    [Serializable]
    public class CuttingHeadMotionController : MonoBehaviour
    {
        [Header("目标路径")]
        public RoadwayProfileGenerator profileGenerator;
        public BezierSpline targetSpline;

        [Header("运动参数")]
        [Tooltip("沿路径运动速度 (m/s)")]
        public double traverseSpeed = 0.5;

        [Tooltip("截割头自旋速度 (rad/s)")]
        public double spinSpeed = 10.0;

        [Tooltip("运动平滑时间 (s)")]
        public double smoothingTime = 0.1;

        [Tooltip("最大速度 (m/s)")]
        public double maxSpeed = 2.0;

        [Header("方向约束")]
        [Tooltip("前向轴 (机体坐标系)")]
        public Vector3 forwardAxis = new Vector3(0, 0, 1);

        [Tooltip("向上轴 (机体坐标系)")]
        public Vector3 upAxis = new Vector3(0, 1, 0);

        [Tooltip("是否跟随曲线切线方向")]
        public bool followTangent = true;

        [Tooltip("方向跟随权重")]
        [Range(0, 1)]
        public double orientationWeight = 1.0;

        [Header("数值稳定器")]
        [Tooltip("最大允许偏离样条距离 (m)，超过强制拉回")]
        public double maxDeviationDistance = 0.5;

        [Tooltip("单步最大位移 (m)，防止一帧穿透")]
        public double maxStepDistance = 0.05;

        [Tooltip("自旋速度硬上限 (rad/s)")]
        public double maxSpinSpeed = 50.0;

        [Header("状态")]
        [SerializeField]
        private double _currentDistance;
        [SerializeField]
        private double _currentSpeed;
        [SerializeField]
        private bool _isMoving;
        [SerializeField]
        private bool _isSpinning;

        private Vector3d _smoothedPosition;
        private Quaterniond _smoothedRotation;
        private Quaterniond _spinAccumulator;
        private Vector3d _positionVelocity;
        private double _angularVelocity;
        private Vector3d _lastValidPosition;
        private Quaterniond _lastValidRotation;
        private double _totalSpinAngle;

        public event Action<double> OnDistanceChanged;
        public event Action<bool> OnMotionStateChanged;

        public double CurrentDistance => _currentDistance;
        public double CurrentSpeed => _currentSpeed;
        public double CurrentSpinAngle => _totalSpinAngle;
        public bool IsMoving => _isMoving;
        public bool IsSpinning => _isSpinning;
        public Vector3d SmoothedPosition => _smoothedPosition;
        public Quaterniond SmoothedRotation => _smoothedRotation;
        public Quaterniond SpinAccumulator => _spinAccumulator;

        private Transform _transform;
        private bool _initialized;

        private void Awake()
        {
            _transform = transform;
            Initialize();
        }

        public void Initialize()
        {
            if (_initialized) return;

            Vector3d worldPos = Vector3d.FromVector3(_transform.position);
            Quaterniond worldRot = Quaterniond.FromQuaternion(_transform.rotation);

            _smoothedPosition = worldPos;
            _smoothedRotation = worldRot;
            _spinAccumulator = Quaterniond.Identity;
            _totalSpinAngle = 0;
            _positionVelocity = Vector3d.Zero;
            _angularVelocity = 0;
            _currentDistance = 0;
            _currentSpeed = 0;
            _lastValidPosition = worldPos;
            _lastValidRotation = worldRot;
            _initialized = true;
        }

        public void SetSpline(BezierSpline spline)
        {
            targetSpline = spline;
            _currentDistance = 0;
            UpdateTransform(0, 0.001);
        }

        public void StartMotion()
        {
            _isMoving = true;
            OnMotionStateChanged?.Invoke(true);
        }

        public void StopMotion()
        {
            _isMoving = false;
            _currentSpeed = 0;
            OnMotionStateChanged?.Invoke(false);
        }

        public void StartSpin()
        {
            _isSpinning = true;
        }

        public void StopSpin()
        {
            _isSpinning = false;
        }

        public void ResetMotion()
        {
            _currentDistance = 0;
            _currentSpeed = 0;
            _isMoving = false;
            _isSpinning = false;
            _spinAccumulator = Quaterniond.Identity;
            _totalSpinAngle = 0;
            _positionVelocity = Vector3d.Zero;
            _angularVelocity = 0;
            UpdateTransform(0, 0.001);
            _lastValidPosition = _smoothedPosition;
            _lastValidRotation = _smoothedRotation;
        }

        public void SetDistance(double distance)
        {
            if (targetSpline == null) return;
            _currentDistance = Mathd.Clamp(distance, 0, targetSpline.TotalLength);
            UpdateTransform(0, 0.001);
            OnDistanceChanged?.Invoke(_currentDistance);
        }

        public void UpdateKinematics(double deltaTime)
        {
            if (!_initialized) Initialize();
            if (targetSpline == null) return;
            if (deltaTime <= 0) deltaTime = 0.001;

            double clampedDt = Mathd.Min(deltaTime, Time.maximumDeltaTime);

            if (_isMoving)
            {
                _currentSpeed = Mathd.SmoothDamp(_currentSpeed, traverseSpeed, ref _angularVelocity, smoothingTime, maxSpeed, clampedDt);

                double distanceStep = _currentSpeed * clampedDt;
                distanceStep = Mathd.Clamp(distanceStep, -maxStepDistance, maxStepDistance);
                _currentDistance += distanceStep;

                if (_currentDistance >= targetSpline.TotalLength)
                {
                    if (targetSpline.IsLoop)
                    {
                        _currentDistance -= targetSpline.TotalLength;
                    }
                    else
                    {
                        _currentDistance = targetSpline.TotalLength;
                        _currentSpeed = 0;
                        _isMoving = false;
                        OnMotionStateChanged?.Invoke(false);
                    }
                }

                OnDistanceChanged?.Invoke(_currentDistance);
            }
            else
            {
                _currentSpeed = Mathd.SmoothDamp(_currentSpeed, 0, ref _angularVelocity, smoothingTime * 0.5, maxSpeed, clampedDt);
            }

            if (_isSpinning)
            {
                double clampedSpin = Mathd.Clamp(spinSpeed, -maxSpinSpeed, maxSpinSpeed);
                double spinStep = clampedSpin * clampedDt;
                _totalSpinAngle += spinStep;
                _totalSpinAngle = Mathd.Repeat(_totalSpinAngle, Mathd.TwoPI);
                Vector3d tangent = targetSpline.GetTangentAtDistance(_currentDistance);
                if (tangent.SqrMagnitude > 1e-15)
                {
                    tangent = tangent.Normalized;
                    Quaterniond deltaSpin = Quaterniond.AxisAngle(tangent, spinStep);
                    _spinAccumulator = (deltaSpin * _spinAccumulator).Normalized;
                }
            }

            UpdateTransform(clampedDt, clampedDt);
        }

        private void UpdateTransform(double kinematicsDt, double smoothingDt)
        {
            if (targetSpline == null) return;
            if (smoothingDt <= 0) smoothingDt = 0.001;

            Vector3d targetPos = targetSpline.GetPointAtDistance(_currentDistance);
            Vector3d tangent = targetSpline.GetTangentAtDistance(_currentDistance);
            Vector3d up = Vector3d.FromVector3(upAxis);

            if (tangent.SqrMagnitude < 1e-15)
            {
                tangent = Vector3d.Forward;
            }
            tangent = tangent.Normalized;

            Quaterniond targetRot;
            if (followTangent)
            {
                Vector3d forward = Vector3d.FromVector3(forwardAxis);
                Quaterniond baseRot = Quaterniond.LookRotation(tangent, up);
                double slerpT = Mathd.Clamp01(orientationWeight);
                targetRot = Quaterniond.Slerp(_smoothedRotation, baseRot, slerpT);
            }
            else
            {
                targetRot = Quaterniond.Identity;
            }

            targetRot = (targetRot * _spinAccumulator).Normalized;

            Vector3d newPosition = SmoothDampVector(_smoothedPosition, targetPos, ref _positionVelocity, smoothingTime, maxSpeed, smoothingDt);

            Vector3d positionDelta = newPosition - _smoothedPosition;
            double deltaMag = positionDelta.Magnitude;
            if (deltaMag > maxStepDistance)
            {
                newPosition = _smoothedPosition + positionDelta.Normalized * maxStepDistance;
            }

            double deviation = Vector3d.Distance(newPosition, targetPos);
            if (deviation > maxDeviationDistance)
            {
                newPosition = targetPos + (newPosition - targetPos).Normalized * maxDeviationDistance;
                _positionVelocity = Vector3d.Zero;
            }

            if (double.IsNaN(newPosition.x) || double.IsNaN(newPosition.y) || double.IsNaN(newPosition.z) ||
                double.IsInfinity(newPosition.x) || double.IsInfinity(newPosition.y) || double.IsInfinity(newPosition.z))
            {
                newPosition = _lastValidPosition;
                _positionVelocity = Vector3d.Zero;
            }

            Quaterniond newRotation = Quaterniond.Slerp(_smoothedRotation, targetRot, Mathd.Clamp01(smoothingDt / smoothingTime));
            newRotation = newRotation.Normalized;

            if (double.IsNaN(newRotation.x) || double.IsNaN(newRotation.y) || double.IsNaN(newRotation.z) || double.IsNaN(newRotation.w) ||
                double.IsInfinity(newRotation.x) || double.IsInfinity(newRotation.y) || double.IsInfinity(newRotation.z) || double.IsInfinity(newRotation.w))
            {
                newRotation = _lastValidRotation;
            }

            _smoothedPosition = newPosition;
            _smoothedRotation = newRotation;
            _lastValidPosition = newPosition;
            _lastValidRotation = newRotation;

            _transform.position = _smoothedPosition.ToVector3();
            _transform.rotation = _smoothedRotation.ToQuaternion();
        }

        private Vector3d SmoothDampVector(Vector3d current, Vector3d target, ref Vector3d currentVelocity, double smoothTime, double maxSpeed, double deltaTime)
        {
            Vector3d result = new Vector3d(
                Mathd.SmoothDamp(current.x, target.x, ref currentVelocity.x, smoothTime, maxSpeed, deltaTime),
                Mathd.SmoothDamp(current.y, target.y, ref currentVelocity.y, smoothTime, maxSpeed, deltaTime),
                Mathd.SmoothDamp(current.z, target.z, ref currentVelocity.z, smoothTime, maxSpeed, deltaTime)
            );
            return result;
        }

        public Vector3d GetTangent()
        {
            if (targetSpline == null) return Vector3d.Forward;
            Vector3d tangent = targetSpline.GetTangentAtDistance(_currentDistance);
            if (tangent.SqrMagnitude < 1e-15) return Vector3d.Forward;
            return tangent.Normalized;
        }

        public Vector3d GetNormal()
        {
            if (targetSpline == null) return Vector3d.Up;
            return targetSpline.GetNormal(_currentDistance / Mathd.Max(targetSpline.TotalLength, 1e-10), Vector3d.FromVector3(upAxis));
        }

        public List<Vector3d> GetPathPreview(int samples = 100)
        {
            if (targetSpline == null) return new List<Vector3d>();
            return targetSpline.GetPoints(samples / Math.Max(1, targetSpline.SegmentCount));
        }

        private void OnDrawGizmosSelected()
        {
            if (targetSpline == null) return;

            Gizmos.color = Color.green;
            var points = GetPathPreview(50);
            for (int i = 0; i < points.Count - 1; i++)
            {
                Gizmos.DrawLine(points[i].ToVector3(), points[i + 1].ToVector3());
            }

            if (_initialized)
            {
                Vector3 pos = _smoothedPosition.ToVector3();
                Vector3 tangent = GetTangent().ToVector3();
                Vector3 normal = GetNormal().ToVector3();

                Gizmos.color = Color.red;
                Gizmos.DrawRay(pos, tangent * 0.5f);
                Gizmos.color = Color.blue;
                Gizmos.DrawRay(pos, normal * 0.5f);
                Gizmos.color = Color.yellow;
                Gizmos.DrawSphere(pos, 0.1f);

                Gizmos.color = new Color(1, 0, 0, 0.2f);
                Gizmos.DrawWireSphere(pos, (float)maxDeviationDistance);
            }
        }

        private void FixedUpdate()
        {
            UpdateKinematics(Time.fixedDeltaTime);
        }
    }
}
