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

        [Header("状态")]
        [SerializeField]
        private double _currentDistance;
        [SerializeField]
        private double _currentSpeed;
        [SerializeField]
        private double _currentSpinAngle;
        [SerializeField]
        private bool _isMoving;
        [SerializeField]
        private bool _isSpinning;

        private Vector3d _smoothedPosition;
        private Quaterniond _smoothedRotation;
        private Vector3d _positionVelocity;
        private double _angularVelocity;

        public event Action<double> OnDistanceChanged;
        public event Action<bool> OnMotionStateChanged;

        public double CurrentDistance => _currentDistance;
        public double CurrentSpeed => _currentSpeed;
        public double CurrentSpinAngle => _currentSpinAngle;
        public bool IsMoving => _isMoving;
        public bool IsSpinning => _isSpinning;
        public Vector3d SmoothedPosition => _smoothedPosition;
        public Quaterniond SmoothedRotation => _smoothedRotation;

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
            _positionVelocity = Vector3d.Zero;
            _angularVelocity = 0;
            _currentDistance = 0;
            _currentSpeed = 0;
            _currentSpinAngle = 0;
            _initialized = true;
        }

        public void SetSpline(BezierSpline spline)
        {
            targetSpline = spline;
            _currentDistance = 0;
            UpdateTransform(0, 0);
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
            _currentSpinAngle = 0;
            _isMoving = false;
            _isSpinning = false;
            UpdateTransform(0, 0);
        }

        public void SetDistance(double distance)
        {
            if (targetSpline == null) return;
            _currentDistance = Mathd.Clamp(distance, 0, targetSpline.TotalLength);
            UpdateTransform(_currentDistance, _currentSpinAngle);
            OnDistanceChanged?.Invoke(_currentDistance);
        }

        public void UpdateKinematics(double deltaTime)
        {
            if (!_initialized) Initialize();
            if (targetSpline == null) return;

            if (_isMoving)
            {
                _currentSpeed = Mathd.SmoothDamp(_currentSpeed, traverseSpeed, ref _angularVelocity, smoothingTime, maxSpeed, deltaTime);
                _currentDistance += _currentSpeed * deltaTime;

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
                _currentSpeed = Mathd.SmoothDamp(_currentSpeed, 0, ref _angularVelocity, smoothingTime * 0.5, maxSpeed, deltaTime);
            }

            if (_isSpinning)
            {
                _currentSpinAngle += spinSpeed * deltaTime;
                _currentSpinAngle = Mathd.Repeat(_currentSpinAngle, Mathd.TwoPI);
            }

            UpdateTransform(_currentDistance, _currentSpinAngle);
        }

        private void UpdateTransform(double distance, double spinAngle)
        {
            if (targetSpline == null) return;

            Vector3d targetPos = targetSpline.GetPointAtDistance(distance);
            Vector3d tangent = targetSpline.GetTangentAtDistance(distance);
            Vector3d up = Vector3d.FromVector3(upAxis);

            Quaterniond targetRot;
            if (followTangent)
            {
                Vector3d forward = Vector3d.FromVector3(forwardAxis);
                Quaterniond tangentRot = Quaterniond.FromToRotation(forward, tangent);
                Quaterniond baseRot = Quaterniond.LookRotation(tangent, up);
                targetRot = Quaterniond.Slerp(_smoothedRotation, baseRot, orientationWeight);
            }
            else
            {
                targetRot = Quaterniond.Identity;
            }

            Quaterniond spinRot = Quaterniond.AxisAngle(tangent, spinAngle);
            targetRot = targetRot * spinRot;

            _smoothedPosition = SmoothDampVector(_smoothedPosition, targetPos, ref _positionVelocity, smoothingTime, maxSpeed, Time.deltaTime);
            _smoothedRotation = Quaterniond.Slerp(_smoothedRotation, targetRot, Mathd.Clamp01(Time.deltaTime / smoothingTime));

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
            return targetSpline.GetTangentAtDistance(_currentDistance);
        }

        public Vector3d GetNormal()
        {
            if (targetSpline == null) return Vector3d.Up;
            return targetSpline.GetNormal(_currentDistance / targetSpline.TotalLength, Vector3d.FromVector3(upAxis));
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
            }
        }

        private void Update()
        {
            UpdateKinematics(Time.deltaTime);
        }
    }
}
