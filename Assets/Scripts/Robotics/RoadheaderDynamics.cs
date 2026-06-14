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

            _accumulatedTime += deltaTime;

            int subSteps = Math.Min(maxSubSteps, (int)Math.Ceiling(_accumulatedTime / fixedTimestep));

            for (int i = 0; i < subSteps; i++)
            {
                double stepTime = Math.Min(fixedTimestep, _accumulatedTime);
                _accumulatedTime -= stepTime;

                PhysicsStep(stepTime);
            }
        }

        private void PhysicsStep(double dt)
        {
            if (!_initialized) return;

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
                    ? cuttingHeadController.GetTangent() * cuttingHeadController.spinSpeed
                    : Vector3d.Zero;

                double spinAngle = cuttingHeadController != null
                    ? cuttingHeadController.CurrentSpinAngle
                    : 0;

                collisionSolver.UpdateHeadState(headPos, headRot, headAngVel, spinAngle, dt);

                cuttingResistance = -collisionSolver.totalContactForce;
                totalForce += cuttingResistance;

                Vector3d torqueArm = headPos - bodyState.position;
                totalTorque += Vector3d.Cross(torqueArm, -collisionSolver.totalContactForce);
                totalTorque += collisionSolver.totalContactTorque;
            }

            propulsionForce = CalculatePropulsionForce(dt);
            totalForce += propulsionForce;

            Vector3d groundForce = CalculateGroundContactForce();
            totalForce += groundForce;

            totalForce += externalForce;
            totalTorque += externalTorque;

            Integrate(totalForce, totalTorque, dt);

            if (bodyFrame != null)
            {
                bodyFrame.position = bodyState.position.ToVector3();
                bodyFrame.rotation = bodyState.rotation.ToQuaternion();
            }

            OnPhysicsStep?.Invoke(bodyState);
            OnForcesUpdated?.Invoke(totalForce, totalTorque);
        }

        private Vector3d CalculatePropulsionForce(double dt)
        {
            if (cuttingHeadController == null) return Vector3d.Zero;

            Vector3d desiredDirection = cuttingHeadController.GetTangent();

            double targetSpeed = cuttingHeadController.traverseSpeed * 0.1;
            double speedError = targetSpeed - currentSpeed;

            double forceMagnitude = speedError * propulsionGain * bodyState.mass;
            forceMagnitude = Mathd.Clamp(forceMagnitude, -maxPropulsionForce, maxPropulsionForce);

            Vector3d propulsion = desiredDirection * forceMagnitude;

            Vector3d dampingForce = -bodyState.linearVelocity * propulsionDamping;

            return propulsion + dampingForce;
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

                for (int i = 0; i < 4; i++)
                {
                    if (trackY[i] < 0)
                    {
                        double normalForce = -trackY[i] * stiffness;
                        normalForce = Mathd.Min(normalForce, bodyState.mass * gravity * 0.5);

                        double verticalVel = Vector3d.Dot(bodyState.linearVelocity, Vector3d.Up);
                        if (verticalVel < 0)
                        {
                            normalForce += -verticalVel * damping;
                        }

                        force += Vector3d.Up * normalForce;

                        Vector3d lever = trackPoints[i] - comWorld;
                        torque += Vector3d.Cross(lever, Vector3d.Up * normalForce);
                    }
                }

                Vector3d lateralVel = bodyState.linearVelocity - bodyUp * Vector3d.Dot(bodyState.linearVelocity, bodyUp);
                double frictionCoeff = 0.8;
                double maxFriction = bodyState.mass * gravity * frictionCoeff;
                double lateralForceMag = Mathd.Min(lateralVel.Magnitude * 1e5, maxFriction);
                force += -lateralVel.Normalized * lateralForceMag;
            }

            externalTorque += torque;
            currentSpeed = Vector3d.Dot(bodyState.linearVelocity, bodyForward);
            currentAdvanceRate = Vector3d.Dot(bodyState.linearVelocity, bodyForward) * 3600.0 / 1000.0;

            return force;
        }

        private void Integrate(Vector3d force, Vector3d torque, double dt)
        {
            bodyState.linearAcceleration = force / bodyState.mass;
            bodyState.linearVelocity += bodyState.linearAcceleration * dt;
            bodyState.position += bodyState.linearVelocity * dt;

            Matrix4x4d worldInertia = bodyState.rotation.ToMatrix4x4d() *
                                      bodyState.inertiaTensor *
                                      bodyState.rotation.ToMatrix4x4d().Inverse;

            bodyState.angularAcceleration = worldInertia.MultiplyVector(torque);
            bodyState.angularVelocity += bodyState.angularAcceleration * dt;

            double angle = bodyState.angularVelocity.Magnitude * dt;
            if (angle > 1e-12)
            {
                Vector3d axis = bodyState.angularVelocity.Normalized;
                Quaterniond deltaRot = Quaterniond.AxisAngle(axis, angle);
                bodyState.rotation = (deltaRot * bodyState.rotation).Normalized;
            }

            double maxVelocity = 5.0;
            if (bodyState.linearVelocity.Magnitude > maxVelocity)
            {
                bodyState.linearVelocity = bodyState.linearVelocity.Normalized * maxVelocity;
            }

            double maxAngularVelocity = 10.0;
            if (bodyState.angularVelocity.Magnitude > maxAngularVelocity)
            {
                bodyState.angularVelocity = bodyState.angularVelocity.Normalized * maxAngularVelocity;
            }
        }

        public void ApplyExternalForce(Vector3d force, Vector3d position)
        {
            externalForce += force;
            Vector3d lever = position - bodyState.position;
            externalTorque += Vector3d.Cross(lever, force);
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

            externalForce = Vector3d.Zero;
            externalTorque = Vector3d.Zero;
            currentSpeed = 0;
            currentAdvanceRate = 0;

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
