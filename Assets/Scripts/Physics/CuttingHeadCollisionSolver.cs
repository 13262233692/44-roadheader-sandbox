using System;
using System.Collections.Generic;
using RoadheaderSandbox.Core.Math;
using RoadheaderSandbox.Robotics;
using UnityEngine;

namespace RoadheaderSandbox.Physics
{
    [Serializable]
    public class CuttingHeadCollisionSolver : MonoBehaviour
    {
        [Header("引用")]
        public RockSurface rockSurface;
        public List<Pick> picks = new List<Pick>();

        [Header("截割头参数")]
        [Tooltip("截割头半径 (m)")]
        public double headRadius = 0.4;

        [Tooltip("截割头长度 (m)")]
        public double headLength = 0.8;

        [Header("物理求解")]
        public HertzContactSolver contactSolver = new HertzContactSolver();

        [Header("开挖参数")]
        [Tooltip("最小开挖力阈值 (N)")]
        public double excavationForceThreshold = 1000.0;

        [Tooltip("开挖效率系数")]
        public double excavationEfficiency = 0.7;

        [Header("状态")]
        public Vector3d totalContactForce;
        public Vector3d totalContactTorque;
        public double totalExcavationVolume;
        public double specificEnergy;
        public int activeContactCount;

        [Header("调试")]
        public bool drawContactForces = true;
        public double forceDrawScale = 0.0001;

        [Header("硬性截断钳制")]
        [Tooltip("单齿接触力硬上限 (N)")]
        public double maxPickContactForce = 5.0e4;

        [Tooltip("单齿穿透深度硬上限 (m)")]
        public double maxPickPenetration = 0.02;

        [Tooltip("截割头总合力硬上限 (N)")]
        public double maxTotalContactForce = 2.0e6;

        [Tooltip("截割头总扭矩硬上限 (N·m)")]
        public double maxTotalContactTorque = 1.0e6;

        public event Action<int, ContactForce> OnPickContact;
        public event Action<Vector3d, Vector3d> OnTotalForceUpdated;
        public event Action<double, double> OnExcavationUpdated;

        private Matrix4x4d _headTransform;
        private Vector3d _headAngularVelocity;
        private double _headSpinAngle;
        private Vector3d _headPosition;
        private Quaterniond _headRotation;

        private void Awake()
        {
            if (rockSurface == null)
            {
                rockSurface = FindObjectOfType<RockSurface>();
            }

            if (contactSolver == null)
            {
                contactSolver = new HertzContactSolver();
            }

            CollectPicks();
        }

        public void CollectPicks()
        {
            picks.Clear();
            GetComponentsInChildren<Pick>(picks);
        }

        public void AddPick(Pick pick)
        {
            if (!picks.Contains(pick))
            {
                picks.Add(pick);
            }
        }

        public void RemovePick(Pick pick)
        {
            picks.Remove(pick);
        }

        public void UpdateHeadState(
            Vector3d position,
            Quaterniond rotation,
            Vector3d angularVelocity,
            double spinAngle,
            double deltaTime)
        {
            _headPosition = position;
            _headRotation = rotation;
            _headAngularVelocity = angularVelocity;
            _headSpinAngle = spinAngle;

            _headTransform = Matrix4x4d.TRS(position, rotation, Vector3d.One);

            foreach (var pick in picks)
            {
                pick.UpdateState(_headTransform, angularVelocity, spinAngle, deltaTime);
            }

            SolveCollisions(deltaTime);
        }

        private void SolveCollisions(double deltaTime)
        {
            totalContactForce = Vector3d.Zero;
            totalContactTorque = Vector3d.Zero;
            activeContactCount = 0;
            double stepExcavationVolume = 0;

            foreach (var pick in picks)
            {
                pick.ResetContactState();

                if (rockSurface == null || pick.material == null || rockSurface.material == null)
                    continue;

                Vector3d tipPos = pick.tipPosition;
                Vector3d tipVel = pick.tipVelocity;
                double pickRadius = pick.tipRadius;

                ContactPoint contactPoint = rockSurface.CheckCollision(tipPos, pickRadius, tipVel);

                if (contactPoint.penetrationDepth > 0)
                {
                    contactPoint.penetrationDepth = Mathd.Min(contactPoint.penetrationDepth, maxPickPenetration);

                    var geometry = contactSolver.CalculateEquivalentGeometry(
                        pickRadius, double.PositiveInfinity, contactPoint.normal);

                    var material = contactSolver.CalculateEquivalentMaterial(
                        pick.material, rockSurface.material);

                    ContactForce contactForce = contactSolver.SolveContact(
                        contactPoint, geometry, material,
                        pick.material, rockSurface.material, deltaTime);

                    double forceMag = contactForce.totalForce.Magnitude;
                    if (forceMag > maxPickContactForce && forceMag > 1e-15)
                    {
                        double ratio = maxPickContactForce / forceMag;
                        contactForce.totalForce *= ratio;
                        contactForce.normalForce *= ratio;
                        contactForce.tangentialForce *= ratio;
                        contactForce.frictionForce *= ratio;
                        contactForce.normalStress *= ratio;
                        contactForce.shearStress *= ratio;
                    }

                    if (double.IsNaN(contactForce.totalForce.x) || double.IsInfinity(contactForce.totalForce.x) ||
                        double.IsNaN(contactForce.totalForce.y) || double.IsInfinity(contactForce.totalForce.y) ||
                        double.IsNaN(contactForce.totalForce.z) || double.IsInfinity(contactForce.totalForce.z))
                    {
                        continue;
                    }

                    if (contactForce.totalForce.Magnitude > 0)
                    {
                        pick.ApplyContactForce(contactForce, deltaTime);
                        pick.penetrationDepth = contactPoint.penetrationDepth;

                        Vector3d force = contactForce.totalForce;
                        totalContactForce += force;

                        Vector3d lever = tipPos - _headPosition;
                        totalContactTorque += Vector3d.Cross(lever, force);

                        activeContactCount++;

                        OnPickContact?.Invoke(picks.IndexOf(pick), contactForce);

                        if (contactForce.normalForce.Magnitude > excavationForceThreshold)
                        {
                            double excavated = CalculateExcavationVolume(pick, contactForce, deltaTime);
                            stepExcavationVolume += excavated;
                            totalExcavationVolume += excavated;

                            double excavRadius = Mathd.Pow(3.0 * excavated / (4.0 * Mathd.PI), 1.0 / 3.0);
                            if (excavRadius > 0.001)
                            {
                                rockSurface.ExcavateSphere(tipPos, excavRadius);
                            }
                        }
                    }
                }
            }

            double totalForceMag = totalContactForce.Magnitude;
            if (totalForceMag > maxTotalContactForce && totalForceMag > 1e-15)
            {
                totalContactForce = totalContactForce.Normalized * maxTotalContactForce;
            }

            if (double.IsNaN(totalContactForce.x) || double.IsInfinity(totalContactForce.x) ||
                double.IsNaN(totalContactForce.y) || double.IsInfinity(totalContactForce.y) ||
                double.IsNaN(totalContactForce.z) || double.IsInfinity(totalContactForce.z))
            {
                totalContactForce = Vector3d.Zero;
            }

            double totalTorqueMag = totalContactTorque.Magnitude;
            if (totalTorqueMag > maxTotalContactTorque && totalTorqueMag > 1e-15)
            {
                totalContactTorque = totalContactTorque.Normalized * maxTotalContactTorque;
            }

            if (double.IsNaN(totalContactTorque.x) || double.IsInfinity(totalContactTorque.x) ||
                double.IsNaN(totalContactTorque.y) || double.IsInfinity(totalContactTorque.y) ||
                double.IsNaN(totalContactTorque.z) || double.IsInfinity(totalContactTorque.z))
            {
                totalContactTorque = Vector3d.Zero;
            }

            OnTotalForceUpdated?.Invoke(totalContactForce, totalContactTorque);

            if (stepExcavationVolume > 0)
            {
                double power = Vector3d.Dot(totalContactForce, _headAngularVelocity) * headRadius;
                specificEnergy = power > 0 ? power / stepExcavationVolume : 0;
                OnExcavationUpdated?.Invoke(stepExcavationVolume, specificEnergy);
            }
        }

        private double CalculateExcavationVolume(Pick pick, ContactForce force, double deltaTime)
        {
            if (rockSurface == null || rockSurface.material == null) return 0;

            double normalForce = force.normalForce.Magnitude;
            double compressiveStrength = rockSurface.material.compressiveStrength;

            if (compressiveStrength <= 0) return 0;

            double attackAngle = pick.installationAngle * Mathd.Deg2Rad;
            double projectedArea = pick.GetProjectedArea(attackAngle);
            double cuttingSpeed = pick.tipVelocity.Magnitude;

            double theoreticalVolume = (normalForce / compressiveStrength) * cuttingSpeed * deltaTime;

            double penetrationFactor = Mathd.Clamp01(pick.penetrationDepth / (pick.tipRadius * 2.0));
            double efficiencyFactor = excavationEfficiency * penetrationFactor;

            return theoreticalVolume * efficiencyFactor;
        }

        public List<ContactForce> GetAllContactForces()
        {
            List<ContactForce> forces = new List<ContactForce>();
            foreach (var pick in picks)
            {
                if (pick.isInContact)
                {
                    forces.Add(new ContactForce
                    {
                        totalForce = pick.contactForce,
                        normalForce = pick.contactForce,
                        tangentialForce = Vector3d.Zero,
                        normalStress = 0,
                        shearStress = 0,
                        frictionForce = pick.contactForceMagnitude,
                        creepForce = 0
                    });
                }
            }
            return forces;
        }

        public double GetAverageContactForce()
        {
            if (activeContactCount == 0) return 0;
            return totalContactForce.Magnitude / activeContactCount;
        }

        public double GetMaximumContactForce()
        {
            double maxForce = 0;
            foreach (var pick in picks)
            {
                if (pick.isInContact && pick.contactForceMagnitude > maxForce)
                {
                    maxForce = pick.contactForceMagnitude;
                }
            }
            return maxForce;
        }

        public double GetTotalWearVolume()
        {
            double totalWear = 0;
            foreach (var pick in picks)
            {
                totalWear += pick.wearVolume;
            }
            return totalWear;
        }

        public void ResetSimulation()
        {
            totalContactForce = Vector3d.Zero;
            totalContactTorque = Vector3d.Zero;
            totalExcavationVolume = 0;
            specificEnergy = 0;
            activeContactCount = 0;

            foreach (var pick in picks)
            {
                pick.ResetContactState();
                pick.wearVolume = 0;
                pick.temperature = 0;
            }

            if (rockSurface != null)
            {
                rockSurface.excavatedRegions.Clear();
                rockSurface.InitializeHeightField();
            }
        }

        private void OnDrawGizmos()
        {
            if (!drawContactForces) return;

            foreach (var pick in picks)
            {
                if (pick.isInContact)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawRay(
                        pick.tipPosition.ToVector3(),
                        pick.contactForce.ToVector3() * (float)forceDrawScale
                    );

                    Gizmos.color = Color.yellow;
                    Gizmos.DrawWireSphere(pick.tipPosition.ToVector3(), (float)(pick.tipRadius * 1.5));
                }
                else
                {
                    Gizmos.color = Color.blue;
                    Gizmos.DrawWireSphere(pick.tipPosition.ToVector3(), (float)pick.tipRadius);
                }
            }

            if (_headTransform != null)
            {
                Gizmos.color = new Color(1, 0.5f, 0, 0.3f);
                Vector3 pos = _headPosition.ToVector3();
                Quaternion rot = _headRotation.ToQuaternion();
                Vector3 scale = new Vector3((float)(headRadius * 2), (float)(headRadius * 2), (float)headLength);
                Gizmos.matrix = Matrix4x4.TRS(pos, rot, scale);
                Gizmos.DrawCube(Vector3.zero, Vector3.one);
                Gizmos.matrix = Matrix4x4.identity;
            }
        }
    }
}
