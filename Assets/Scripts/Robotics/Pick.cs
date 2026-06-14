using System;
using RoadheaderSandbox.Core.Math;
using RoadheaderSandbox.Physics;
using UnityEngine;

namespace RoadheaderSandbox.Robotics
{
    [Serializable]
    public class Pick : MonoBehaviour
    {
        [Header("几何参数")]
        [Tooltip("齿尖半径 (m)")]
        public double tipRadius = 0.008;

        [Tooltip("齿身半径 (m)")]
        public double bodyRadius = 0.015;

        [Tooltip("齿总长度 (m)")]
        public double totalLength = 0.08;

        [Tooltip("齿尖锥角 (度)")]
        public double tipAngle = 75.0;

        [Header("安装参数")]
        [Tooltip("安装角 (度)")]
        public double installationAngle = 45.0;

        [Tooltip("在截割头上的周向角度 (度)")]
        public double circumferentialAngle = 0.0;

        [Tooltip("在截割头上的轴向位置 (m)")]
        public double axialPosition = 0.0;

        [Tooltip("截割头半径方向位置 (m)")]
        public double radialPosition = 0.4;

        [Header("材料属性")]
        public MaterialProperties material = null;

        [Header("状态")]
        public Vector3d globalPosition;
        public Vector3d globalVelocity;
        public Vector3d globalAngularVelocity;
        public Vector3d tipPosition;
        public Vector3d tipVelocity;

        [Header("接触状态")]
        public bool isInContact;
        public double contactForceMagnitude;
        public Vector3d contactForce;
        public double penetrationDepth;
        public double wearVolume;
        public double temperature;

        public event Action<Pick, ContactForce> OnPickContact;

        private Transform _transform;
        private Matrix4x4d _localToWorld;

        private void Awake()
        {
            _transform = transform;
            if (material == null)
            {
                material = MaterialProperties.TungstenCarbide();
            }
        }

        public void UpdateState(Matrix4x4d headTransform, Vector3d headAngularVelocity, double headSpinAngle, double deltaTime)
        {
            if (_transform == null) _transform = transform;

            double circumRad = circumferentialAngle * Mathd.Deg2Rad + headSpinAngle;
            double installRad = installationAngle * Mathd.Deg2Rad;

            Vector3d localPosition = new Vector3d(
                Mathd.Cos(circumRad) * radialPosition,
                Mathd.Sin(circumRad) * radialPosition,
                axialPosition
            );

            Quaterniond localRotation =
                Quaterniond.AxisAngle(Vector3d.Forward, circumRad) *
                Quaterniond.AxisAngle(Vector3d.Right, installRad);

            _localToWorld = headTransform * Matrix4x4d.TRS(localPosition, localRotation, Vector3d.One);

            Vector3d localTipOffset = new Vector3d(0, 0, totalLength * 0.5);
            globalPosition = _localToWorld.MultiplyPoint(Vector3d.Zero);
            tipPosition = _localToWorld.MultiplyPoint(localTipOffset);

            Vector3d relativePos = globalPosition - headTransform.MultiplyPoint(Vector3d.Zero);
            globalVelocity = Vector3d.Cross(headAngularVelocity, relativePos);
            tipVelocity = globalVelocity + Vector3d.Cross(headAngularVelocity, tipPosition - globalPosition);

            globalAngularVelocity = headAngularVelocity;

            if (_transform != null)
            {
                _transform.position = globalPosition.ToVector3();
                _transform.rotation = QuaterniondFromMatrix(_localToWorld).ToQuaternion();
            }
        }

        private Quaterniond QuaterniondFromMatrix(Matrix4x4d m)
        {
            double tr = m.m00 + m.m11 + m.m22;
            Quaterniond q = new Quaterniond();

            if (tr > 0)
            {
                double s = Mathd.Sqrt(tr + 1.0) * 2.0;
                q.w = 0.25 * s;
                q.x = (m.m12 - m.m21) / s;
                q.y = (m.m20 - m.m02) / s;
                q.z = (m.m01 - m.m10) / s;
            }
            else if (m.m00 > m.m11 && m.m00 > m.m22)
            {
                double s = Mathd.Sqrt(1.0 + m.m00 - m.m11 - m.m22) * 2.0;
                q.w = (m.m12 - m.m21) / s;
                q.x = 0.25 * s;
                q.y = (m.m10 + m.m01) / s;
                q.z = (m.m20 + m.m02) / s;
            }
            else if (m.m11 > m.m22)
            {
                double s = Mathd.Sqrt(1.0 + m.m11 - m.m00 - m.m22) * 2.0;
                q.w = (m.m20 - m.m02) / s;
                q.x = (m.m10 + m.m01) / s;
                q.y = 0.25 * s;
                q.z = (m.m21 + m.m12) / s;
            }
            else
            {
                double s = Mathd.Sqrt(1.0 + m.m22 - m.m00 - m.m11) * 2.0;
                q.w = (m.m01 - m.m10) / s;
                q.x = (m.m20 + m.m02) / s;
                q.y = (m.m21 + m.m12) / s;
                q.z = 0.25 * s;
            }

            return q.Normalized;
        }

        public void ApplyContactForce(ContactForce force, double deltaTime)
        {
            isInContact = true;
            contactForce = force.totalForce;
            contactForceMagnitude = force.totalForce.Magnitude;

            double wear = 0;
            if (material != null)
            {
                wear = force.frictionForce * globalVelocity.Magnitude * deltaTime / material.hardness;
                wearVolume += wear;
                temperature += wear * 1e-6;
            }

            OnPickContact?.Invoke(this, force);
        }

        public void ResetContactState()
        {
            isInContact = false;
            contactForce = Vector3d.Zero;
            contactForceMagnitude = 0;
            penetrationDepth = 0;
        }

        public Vector3d GetTipDirection()
        {
            return (_localToWorld.MultiplyPoint(new Vector3d(0, 0, 1)) - globalPosition).Normalized;
        }

        public double GetProjectedArea(double attackAngle)
        {
            double sinAngle = Mathd.Sin(attackAngle);
            double projectedWidth = tipRadius * 2.0 * sinAngle;
            double projectedHeight = bodyRadius * 2.0 * sinAngle;
            return projectedWidth * projectedHeight;
        }

        private void OnDrawGizmosSelected()
        {
            if (_transform == null) _transform = transform;

            Gizmos.color = isInContact ? Color.red : Color.blue;
            Gizmos.DrawWireSphere(_transform.position, (float)tipRadius);

            if (isInContact)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawRay(_transform.position, contactForce.ToVector3() * 0.001f);
            }
        }
    }
}
