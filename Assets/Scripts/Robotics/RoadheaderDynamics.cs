using System;
using System.Collections.Generic;
using RoadheaderSandbox.Core.Math;
using RoadheaderSandbox.Kinematics;
using RoadheaderSandbox.Physics;
using UnityEngine;

namespace RoadheaderSandbox.Robotics
{
    [Serializable]
    public class RigidBodyState
    {
        public Vector3d position;
        public Quaterniond rotation;
        public Vector3d linearVelocity;
        public Vector3d angularVelocity;
        public Vector3d linearAcceleration;
        public Vector3d angularAcceleration;
        public double mass;
        public Matrix4x4d inertiaTensor;
        public Matrix4x4d inverseInertiaTensor;

        public RigidBodyState()
        {
            position = Vector3d.Zero;
            rotation = Quaterniond.Identity;
            linearVelocity = Vector3d.Zero;
            angularVelocity = Vector3d.Zero;
            linearAcceleration = Vector3d.Zero;
            angularAcceleration = Vector3d.Zero;
            mass = 1.0;
            inertiaTensor = Matrix4x4d.Identity;
            inverseInertiaTensor = Matrix4x4d.Identity;
        }

        public void SetBoxInertia(double mass, Vector3d size)
        {
            this.mass = mass;
            double x2 = size.x * size.x;
            double y2 = size.y * size.y;
            double z2 = size.z * size.z;

            inertiaTensor = Matrix4x4d.Identity;
            inertiaTensor.m00 = mass * (y2 + z2) / 12.0;
            inertiaTensor.m11 = mass * (x2 + z2) / 12.0;
            inertiaTensor.m22 = mass * (x2 + y2) / 12.0;
            inverseInertiaTensor = inertiaTensor.Inverse;
        }
    }

    [Serializable]
    public class RoadheaderDynamics : MonoBehaviour
    {
        [Header("整机参数")]
        [Tooltip("整机质量 (kg)")]
        public double totalMass = 45000.0;

        [Tooltip("整机尺寸 (m)")]
        public Vector3 machineSize = new Vector3(2.5f, 1.8f, 8.0f);

        [Tooltip("质心位置 (相对于机体原点)")]
        public Vector3 centerOfMass = new Vector3(0, 0.5f, 2.0f);

        [Header("履带参数")]
        [Tooltip("履带宽度 (m)")]
        public double trackWidth = 0.6;

        [Tooltip("履带接地长度 (m)")]
        public double trackLength = 4.0;

        [Tooltip("左右履带间距 (m)")]
        public double trackGauge = 2.0;

        [Header("推进系统")]
        [Tooltip("最大推进力 (N)")]
        public double maxPropulsionForce = 500000.0;

        [Tooltip("推进速度系数")]
        public double propulsionGain = 1e-6;

        [Tooltip("推进阻尼")]
        public double propulsionDamping = 1e5;

        [Header("物理 Tick")]
        [Tooltip("物理步长 (s)")]
        public double fixedTimestep = 0.001;

        [Tooltip("最大子步数")]
        public int maxSubSteps = 10;

        [Header("重力")]
        [Tooltip("重力加速度 (m/s²)")]
        public double gravity = 9.81;

        [Tooltip("是否启用重力")]
        public bool enableGravity = true;

        [Header("数值稳定器 - 硬性截断钳制")]
        [Tooltip("整机合力硬上限 (N)，超过直接截断")]
        public double maxTotalForce = 2.0e6;

        [Tooltip("整机合扭矩硬上限 (N·m)")]
        public double maxTotalTorque = 1.0e6;

        [Tooltip("最大线加速度 (m/s²)，约10g")]
        public double maxLinearAcceleration = 100.0;

        [Tooltip("最大角加速度 (rad/s²)")]
        public double maxAngularAcceleration = 20.0;

        [Tooltip("最大线速度 (m/s)")]
        public double maxLinearVelocity = 5.0;

        [Tooltip("最大角速度 (rad/s)")]
        public double maxAngularVelocity = 10.0;

        [Tooltip("作业区域硬约束 - 机体原点最大活动半径 (m)")]
        public double maxWorldPositionRadius = 50.0;

        [Tooltip("最大允许俯仰角 (deg)，防止倒立")]
        public double maxPitchAngle = 45.0;

        [Tooltip("最大允许侧倾角 (deg)，防止侧翻")]
        public double maxRollAngle = 45.0;

        [Header("系统引用")]
        public CuttingHeadMotionController cuttingHeadController;
        public ArmSkeletonController armController;
        public CuttingHeadCollisionSolver collisionSolver;
        public RockSurface rockSurface;
        public Transform bodyFrame;

        [Header("状态")]
        public RigidBodyState bodyState = new RigidBodyState();
        public Vector3d externalForce;
        public Vector3d externalTorque;
        public Vector3d propulsionForce;
        public Vector3d cuttingResistance;
        public double currentSpeed;
        public double currentAdvanceRate;

        [Header("调试")]
        public bool enableDynamics = true;
        public bool drawForces = true;
        public double forceDrawScale = 0.00001;

        public event Action<RigidBodyState> OnPhysicsStep;
        public event Action<Vector3d, Vector3d> OnForcesUpdated;

        private double _accumulatedTime;
        private bool _initialized;
        private Vector3d _lastValidPosition;
        private Quaterniond _lastValidRotation;

        private void Awake()
        {
            Initialize();
        }

        public void Initialize()
        {
            if (_initialized) return;

            if (bodyFrame == null)
                bodyFrame = transform;

            bodyState = new RigidBodyState();
            bodyState.position = Vector3d.FromVector3(bodyFrame.position);
            bodyState.rotation = Quaterniond.FromQuaternion(bodyFrame.rotation);
            bodyState.SetBoxInertia(totalMass, Vector3d.FromVector3(machineSize));

            _lastValidPosition = bodyState.position;
            _lastValidRotation = bodyState.rotation;

            if (cuttingHeadController == null)
                cuttingHeadController = GetComponentInChildren<CuttingHeadMotionController>();

            if (armController == null)
                armController = GetComponentInChildren<ArmSkeletonController>();

            if (collisionSolver == null)
                collisionSolver = GetComponentInChildren<CuttingHeadCollisionSolver>();

            if (rockSurface == null)
                rockSurface = FindObjectOfType<RockSurface>();

            _initialized = true;
        }

        public void UpdatePhysics(double deltaTime)
        {
            if (!_initialized) Initialize();
            if (!enableDynamics) return;
            if (deltaTime <= 0) deltaTime = 0.001;

            _accumulatedTime += deltaTime;

            int subSteps = Math.Min(maxSubSteps, (int)Math.Ceiling(_accumulatedTime / fixedTimestep));
            subSteps = Math.Max(1, subSteps);

            for (int i = 0; i < subSteps; i++)
            {
                double stepTime = Math.Min(fixedTimestep, _accumulatedTime);
                if (stepTime <= 0) stepTime = 0.001;
                _accumulatedTime -= stepTime;

                PhysicsStep(stepTime);
            }
        }

        private void PhysicsStep(double dt)
        {
            if (!_initialized) return;
            if (dt <= 0) dt = 0.001;

            Vector3d totalForce = Vector3d.Zero;
            Vector3d totalTorque = Vector3d.Zero;

            if (enableGravity)
            {
                Vector3d gravityForce = Vector3d.Down * gravity * bodyState.mass;
                totalForce += gravityForce;
            }

            if (collisionSolver != null)
            {
                Vector3d headPos = cuttingHeadController != null
                    ? cuttingHeadController.SmoothedPosition
                    : bodyState.position + new Vector3d(0, 1.5, -3.0);

                Quaterniond headRot = cuttingHeadController != null
                    ? cuttingHeadController.SmoothedRotation
                    : bodyState.rotation;

                Vector3d headAngVel = cuttingHeadController != null
                    ? cuttingHeadController.GetTangent() * Mathd.Clamp(cuttingHeadController.spinSpeed, -50.0, 50.0)
                    : Vector3d.Zero;

                double spinAngle = cuttingHeadController != null
                    ? cuttingHeadController.CurrentSpinAngle
                    : 0;

                collisionSolver.UpdateHeadState(headPos, headRot, headAngVel, spinAngle, dt);

                cuttingResistance = -collisionSolver.totalContactForce;
                if (double.IsNaN(cuttingResistance.x) || double.IsNaN(cuttingResistance.y) || double.IsNaN(cuttingResistance.z) ||
                    double.IsInfinity(cuttingResistance.x) || double.IsInfinity(cuttingResistance.y) || double.IsInfinity(cuttingResistance.z))
                {
                    cuttingResistance = Vector3d.Zero;
                }
                totalForce += cuttingResistance;

                Vector3d torqueArm = headPos - bodyState.position;
                double torqueMagLimit = maxTotalTorque * 0.5;
                Vector3d contactTorque = Vector3d.Cross(torqueArm, -collisionSolver.totalContactForce);
                contactTorque = ClampVectorMagnitude(contactTorque, torqueMagLimit);
                totalTorque += contactTorque;

                Vector3d solverTorque = ClampVectorMagnitude(collisionSolver.totalContactTorque, torqueMagLimit);
                totalTorque += solverTorque;
            }

            propulsionForce = CalculatePropulsionForce(dt);
            totalForce += propulsionForce;

            Vector3d groundForce = CalculateGroundContactForce();
            totalForce += groundForce;

            totalForce += externalForce;
            totalTorque += externalTorque;

            totalForce = ClampVectorMagnitude(totalForce, maxTotalForce);
            totalTorque = ClampVectorMagnitude(totalTorque, maxTotalTorque);

            Integrate(totalForce, totalTorque, dt);
            EnforcePoseHardLimits();

            if (bodyFrame != null)
            {
                bodyFrame.position = bodyState.position.ToVector3();
                bodyFrame.rotation = bodyState.rotation.ToQuaternion();
            }

            externalForce = Vector3d.Zero;
            externalTorque = Vector3d.Zero;

            OnPhysicsStep?.Invoke(bodyState);
            OnForcesUpdated?.Invoke(totalForce, totalTorque);
        }

        private Vector3d ClampVectorMagnitude(Vector3d vec, double maxMag)
        {
            double mag = vec.Magnitude;
            if (mag > maxMag && mag > 1e-15)
            {
                return vec.Normalized * maxMag;
            }
            if (double.IsNaN(vec.x) || double.IsNaN(vec.y) || double.IsNaN(vec.z) ||
                double.IsInfinity(vec.x) || double.IsInfinity(vec.y) || double.IsInfinity(vec.z))
            {
                return Vector3d.Zero;
            }
            return vec;
        }

        private Vector3d CalculatePropulsionForce(double dt)
        {
            if (cuttingHeadController == null) return Vector3d.Zero;

            Vector3d desiredDirection = cuttingHeadController.GetTangent();
            if (desiredDirection.SqrMagnitude < 1e-15)
            {
                desiredDirection = Vector3d.Forward;
            }
            desiredDirection = desiredDirection.Normalized;

            double targetSpeed = cuttingHeadController.traverseSpeed * 0.1;
            double speedError = targetSpeed - currentSpeed;

            double forceMagnitude = speedError * propulsionGain * bodyState.mass;
            forceMagnitude = Mathd.Clamp(forceMagnitude, -maxPropulsionForce, maxPropulsionForce);

            Vector3d propulsion = desiredDirection * forceMagnitude;
            Vector3d dampingForce = -bodyState.linearVelocity * propulsionDamping;

            Vector3d total = propulsion + dampingForce;
            return ClampVectorMagnitude(total, maxPropulsionForce);
        }

        private Vector3d CalculateGroundContactForce()
        {
            Vector3d force = Vector3d.Zero;
            Vector3d torque = Vector3d.Zero;

            Vector3d bodyUp = bodyState.rotation * Vector3d.Up;
            Vector3d bodyForward = bodyState.rotation * Vector3d.Forward;
            Vector3d bodyRight = bodyState.rotation * Vector3d.Right;

            Vector3d comLocal = Vector3d.FromVector3(centerOfMass);
            Vector3d comWorld = bodyState.position + bodyState.rotation * comLocal;

            double[] trackY = new double[4];
            double groundY = 0;

            Vector3d[] trackPoints = new Vector3d[]
            {
                comWorld - bodyRight * trackGauge * 0.5 - bodyForward * trackLength * 0.5,
                comWorld + bodyRight * trackGauge * 0.5 - bodyForward * trackLength * 0.5,
                comWorld - bodyRight * trackGauge * 0.5 + bodyForward * trackLength * 0.5,
                comWorld + bodyRight * trackGauge * 0.5 + bodyForward * trackLength * 0.5,
            };

            double maxPenetration = 0;
            for (int i = 0; i < 4; i++)
            {
                if (rockSurface != null)
                {
                    groundY = rockSurface.GetHeight(trackPoints[i].x, trackPoints[i].z);
                }
                trackY[i] = trackPoints[i].y - groundY;
                maxPenetration = Mathd.Max(maxPenetration, -trackY[i]);
            }

            if (maxPenetration > 0)
            {
                double stiffness = 1e8;
                double damping = 1e6;
                double maxGroundForce = bodyState.mass * gravity * 2.0;

                for (int i = 0; i < 4; i++)
                {
                    if (trackY[i] < 0)
                    {
                        double normalForce = -trackY[i] * stiffness;
                        normalForce = Mathd.Min(normalForce, maxGroundForce * 0.5);

                        double verticalVel = Vector3d.Dot(bodyState.linearVelocity, Vector3d.Up);
                        if (verticalVel < 0)
                        {
                            normalForce += -verticalVel * damping;
                            normalForce = Mathd.Min(normalForce, maxGroundForce * 0.5);
                        }

                        force += Vector3d.Up * normalForce;

                        Vector3d lever = trackPoints[i] - comWorld;
                        torque += Vector3d.Cross(lever, Vector3d.Up * normalForce);
                    }
                }

                Vector3d lateralVel = bodyState.linearVelocity - bodyUp * Vector3d.Dot(bodyState.linearVelocity, bodyUp);
                double lateralSpeed = lateralVel.Magnitude;
                if (lateralSpeed > 1e-15)
                {
                    double frictionCoeff = 0.8;
                    double maxFriction = bodyState.mass * gravity * frictionCoeff;
                    double lateralForceMag = Mathd.Min(lateralSpeed * 1e5, maxFriction);
                    force += -lateralVel.Normalized * lateralForceMag;
                }
            }

            externalTorque += ClampVectorMagnitude(torque, maxTotalTorque * 0.5);

            currentSpeed = Vector3d.Dot(bodyState.linearVelocity, bodyForward);
            currentAdvanceRate = currentSpeed * 3600.0 / 1000.0;

            return ClampVectorMagnitude(force, maxTotalForce * 0.8);
        }

        private void Integrate(Vector3d force, Vector3d torque, double dt)
        {
            if (dt <= 0) dt = 0.001;

            bodyState.linearAcceleration = force / bodyState.mass;
            bodyState.linearAcceleration = ClampVectorMagnitude(bodyState.linearAcceleration, maxLinearAcceleration);

            bodyState.linearVelocity += bodyState.linearAcceleration * dt;
            bodyState.linearVelocity = ClampVectorMagnitude(bodyState.linearVelocity, maxLinearVelocity);

            Vector3d newPosition = bodyState.position + bodyState.linearVelocity * dt;
            if (double.IsNaN(newPosition.x) || double.IsNaN(newPosition.y) || double.IsNaN(newPosition.z) ||
                double.IsInfinity(newPosition.x) || double.IsInfinity(newPosition.y) || double.IsInfinity(newPosition.z))
            {
                newPosition = _lastValidPosition;
                bodyState.linearVelocity = Vector3d.Zero;
                bodyState.linearAcceleration = Vector3d.Zero;
            }
            bodyState.position = newPosition;

            Matrix4x4d worldInertia;
            try
            {
                worldInertia = bodyState.rotation.ToMatrix4x4d() *
                               bodyState.inertiaTensor *
                               bodyState.rotation.ToMatrix4x4d().Inverse;
            }
            catch
            {
                worldInertia = Matrix4x4d.Identity;
            }

            bodyState.angularAcceleration = worldInertia.MultiplyVector(torque);
            bodyState.angularAcceleration = ClampVectorMagnitude(bodyState.angularAcceleration, maxAngularAcceleration);

            bodyState.angularVelocity += bodyState.angularAcceleration * dt;
            bodyState.angularVelocity = ClampVectorMagnitude(bodyState.angularVelocity, maxAngularVelocity);

            double angle = bodyState.angularVelocity.Magnitude * dt;
            if (angle > 1e-12 && angle < Mathd.PI)
            {
                Vector3d axis = bodyState.angularVelocity.Normalized;
                Quaterniond deltaRot = Quaterniond.AxisAngle(axis, angle);
                Quaterniond newRot = (deltaRot * bodyState.rotation).Normalized;

                if (double.IsNaN(newRot.x) || double.IsNaN(newRot.y) || double.IsNaN(newRot.z) || double.IsNaN(newRot.w) ||
                    double.IsInfinity(newRot.x) || double.IsInfinity(newRot.y) || double.IsInfinity(newRot.z) || double.IsInfinity(newRot.w))
                {
                    newRot = _lastValidRotation;
                    bodyState.angularVelocity = Vector3d.Zero;
                    bodyState.angularAcceleration = Vector3d.Zero;
                }
                bodyState.rotation = newRot;
            }

            _lastValidPosition = bodyState.position;
            _lastValidRotation = bodyState.rotation;
        }

        private void EnforcePoseHardLimits()
        {
            if (bodyState.position.Magnitude > maxWorldPositionRadius)
            {
                bodyState.position = bodyState.position.Normalized * maxWorldPositionRadius;
                bodyState.linearVelocity = Vector3d.Zero;
            }

            Vector3d euler = bodyState.rotation.EulerAngles * Mathd.Deg2Rad;
            double pitchClamped = Mathd.Deg2Rad * maxPitchAngle;
            double rollClamped = Mathd.Deg2Rad * maxRollAngle;
            bool needsClamp = false;

            if (euler.x > pitchClamped || euler.x < -pitchClamped)
            {
                euler.x = Mathd.Clamp(euler.x, -pitchClamped, pitchClamped);
                needsClamp = true;
            }
            if (euler.z > rollClamped || euler.z < -rollClamped)
            {
                euler.z = Mathd.Clamp(euler.z, -rollClamped, rollClamped);
                needsClamp = true;
            }

            if (needsClamp)
            {
                bodyState.rotation = Quaterniond.Euler(euler.x * Mathd.Rad2Deg, euler.y * Mathd.Rad2Deg, euler.z * Mathd.Rad2Deg);
                bodyState.angularVelocity = Vector3d.Zero;
            }

            if (bodyState.position.y < -10.0 || bodyState.position.y > 50.0)
            {
                bodyState.position = _lastValidPosition;
                bodyState.linearVelocity = Vector3d.Zero;
            }
        }

        public void ApplyExternalForce(Vector3d force, Vector3d position)
        {
            Vector3d clampedForce = ClampVectorMagnitude(force, maxTotalForce * 0.5);
            externalForce += clampedForce;

            Vector3d lever = position - bodyState.position;
            Vector3d torque = Vector3d.Cross(lever, clampedForce);
            externalTorque += ClampVectorMagnitude(torque, maxTotalTorque * 0.5);
        }

        public void ResetDynamics()
        {
            if (!_initialized) Initialize();

            bodyState.position = Vector3d.FromVector3(bodyFrame.position);
            bodyState.rotation = Quaterniond.FromQuaternion(bodyFrame.rotation);
            bodyState.linearVelocity = Vector3d.Zero;
            bodyState.angularVelocity = Vector3d.Zero;
            bodyState.linearAcceleration = Vector3d.Zero;
            bodyState.angularAcceleration = Vector3d.Zero;

            _lastValidPosition = bodyState.position;
            _lastValidRotation = bodyState.rotation;

            externalForce = Vector3d.Zero;
            externalTorque = Vector3d.Zero;
            currentSpeed = 0;
            currentAdvanceRate = 0;
            _accumulatedTime = 0;

            if (collisionSolver != null)
            {
                collisionSolver.ResetSimulation();
            }
        }

        private void FixedUpdate()
        {
            UpdatePhysics(Time.fixedDeltaTime);
        }

        private void OnDrawGizmos()
        {
            if (!drawForces || !_initialized) return;

            Gizmos.color = Color.magenta;
            Vector3 com = (bodyState.position + bodyState.rotation * Vector3d.FromVector3(centerOfMass)).ToVector3();
            Gizmos.DrawSphere(com, 0.1f);

            if (propulsionForce.Magnitude > 0)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawRay(com, propulsionForce.ToVector3() * (float)forceDrawScale);
            }

            if (cuttingResistance.Magnitude > 0)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawRay(com, cuttingResistance.ToVector3() * (float)forceDrawScale);
            }

            if (bodyFrame != null)
            {
                Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.3f);
                Gizmos.matrix = Matrix4x4.TRS(
                    bodyState.position.ToVector3(),
                    bodyState.rotation.ToQuaternion(),
                    machineSize
                );
                Gizmos.DrawCube(Vector3.zero, Vector3.one);
                Gizmos.matrix = Matrix4x4.identity;
            }

            Gizmos.color = new Color(1, 0, 0, 0.05f);
            Gizmos.DrawWireSphere(Vector3.zero, (float)maxWorldPositionRadius);
        }
    }

    public static class QuaternionExtensions
    {
        public static Matrix4x4d ToMatrix4x4d(this Quaterniond q)
        {
            return Matrix4x4d.Rotate(q);
        }
    }
}
