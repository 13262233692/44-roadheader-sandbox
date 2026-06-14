using System;
using RoadheaderSandbox.Core.Math;

namespace RoadheaderSandbox.Physics
{
    [Serializable]
    public struct ContactPoint
    {
        public Vector3d position;
        public Vector3d normal;
        public double penetrationDepth;
        public double contactRadius;
        public double contactArea;
        public Vector3d relativeVelocity;
        public Vector3d tangentialVelocity;
        public Vector3d normalVelocity;
    }

    [Serializable]
    public struct ContactForce
    {
        public Vector3d normalForce;
        public Vector3d tangentialForce;
        public Vector3d totalForce;
        public double normalStress;
        public double shearStress;
        public double frictionForce;
        public double creepForce;
    }

    [Serializable]
    public class HertzContactSolver
    {
        [Header("数值精度")]
        public int maxIterations = 50;
        public double tolerance = 1e-12;

        [Header("接触模型")]
        public bool useMindlinTheory = true;
        public bool includeDamping = true;
        public bool includePlasticity = true;

        [Header("蠕滑参数")]
        public double longitudinalCreep = 0.0;
        public double lateralCreep = 0.0;
        public double spinCreep = 0.0;

        [Header("硬性截断钳制")]
        [Tooltip("最大允许穿透深度 (m)，超过强制截断")]
        public double maxPenetrationDepth = 0.02;

        [Tooltip("单接触点最大法向力 (N)")]
        public double maxNormalForce = 5.0e5;

        [Tooltip("单接触点最大切向力 (N)")]
        public double maxTangentialForce = 2.5e5;

        public struct ContactGeometry
        {
            public double radius1;
            public double radius2;
            public Vector3d normal;
            public double curvatureSum;
            public double curvatureDifference;
            public double equivalentRadius;
        }

        public struct ContactMaterial
        {
            public double youngsModulus1;
            public double youngsModulus2;
            public double poissonsRatio1;
            public double poissonsRatio2;
            public double equivalentModulus;
            public double shearModulus1;
            public double shearModulus2;
            public double equivalentShearModulus;
        }

        public ContactMaterial CalculateEquivalentMaterial(MaterialProperties mat1, MaterialProperties mat2)
        {
            ContactMaterial material = new ContactMaterial();
            material.youngsModulus1 = mat1 != null ? mat1.youngsModulus : 210e9;
            material.youngsModulus2 = mat2 != null ? mat2.youngsModulus : 70e9;
            material.poissonsRatio1 = mat1 != null ? mat1.poissonsRatio : 0.3;
            material.poissonsRatio2 = mat2 != null ? mat2.poissonsRatio : 0.25;
            material.shearModulus1 = mat1 != null ? mat1.ShearModulus : 80e9;
            material.shearModulus2 = mat2 != null ? mat2.ShearModulus : 30e9;

            double denom1 = (1.0 - material.poissonsRatio1 * material.poissonsRatio1) / material.youngsModulus1
                          + (1.0 - material.poissonsRatio2 * material.poissonsRatio2) / material.youngsModulus2;
            material.equivalentModulus = denom1 > 1e-30 ? 1.0 / denom1 : 1e30;

            double denom2 = (2.0 - material.poissonsRatio1) / material.shearModulus1
                          + (2.0 - material.poissonsRatio2) / material.shearModulus2;
            material.equivalentShearModulus = denom2 > 1e-30 ? 0.5 / denom2 : 1e30;

            if (double.IsNaN(material.equivalentModulus) || double.IsInfinity(material.equivalentModulus))
                material.equivalentModulus = 1e11;
            if (double.IsNaN(material.equivalentShearModulus) || double.IsInfinity(material.equivalentShearModulus))
                material.equivalentShearModulus = 1e10;

            return material;
        }

        public ContactGeometry CalculateEquivalentGeometry(double radius1, double radius2, Vector3d normal)
        {
            ContactGeometry geometry = new ContactGeometry();
            geometry.radius1 = Mathd.Max(radius1, 0.001);
            geometry.radius2 = Mathd.Max(radius2, 0.001);

            if (normal.SqrMagnitude < 1e-30)
                normal = Vector3d.Up;
            geometry.normal = normal.Normalized;

            if (double.IsPositiveInfinity(radius1) || double.IsPositiveInfinity(radius2))
            {
                geometry.equivalentRadius = Mathd.Min(radius1, radius2);
                if (double.IsPositiveInfinity(geometry.equivalentRadius) || geometry.equivalentRadius <= 0)
                    geometry.equivalentRadius = 0.01;
            }
            else
            {
                geometry.curvatureSum = 1.0 / geometry.radius1 + 1.0 / geometry.radius2;
                geometry.curvatureDifference = Mathd.Abs(1.0 / geometry.radius1 - 1.0 / geometry.radius2);
                if (geometry.curvatureSum > 1e-30)
                    geometry.equivalentRadius = 1.0 / geometry.curvatureSum;
                else
                    geometry.equivalentRadius = 0.01;
            }

            return geometry;
        }

        public double CalculateHertzNormalForce(double penetrationDepth, ContactGeometry geometry, ContactMaterial material)
        {
            if (penetrationDepth <= 0) return 0;

            double delta = Mathd.Clamp(penetrationDepth, 0, maxPenetrationDepth);
            if (delta < 1e-15) return 0;

            double equivR = Mathd.Max(geometry.equivalentRadius, 1e-6);
            double equivE = Mathd.Clamp(material.equivalentModulus, 1e6, 1e13);

            double force = (4.0 / 3.0) * equivE * Mathd.Sqrt(equivR) * Mathd.Pow(delta, 1.5);

            if (double.IsNaN(force) || double.IsInfinity(force))
                force = 0;

            return Mathd.Min(force, maxNormalForce);
        }

        public double CalculateContactRadius(double penetrationDepth, ContactGeometry geometry)
        {
            if (penetrationDepth <= 0) return 0;
            double delta = Mathd.Clamp(penetrationDepth, 0, maxPenetrationDepth);
            double equivR = Mathd.Max(geometry.equivalentRadius, 1e-6);
            double radius = Mathd.Sqrt(equivR * delta);
            if (double.IsNaN(radius) || double.IsInfinity(radius)) return 0;
            return radius;
        }

        public double CalculateContactArea(double contactRadius)
        {
            if (contactRadius <= 0) return 0;
            return Mathd.PI * contactRadius * contactRadius;
        }

        public double CalculateMaximumNormalStress(double normalForce, double contactRadius)
        {
            if (contactRadius <= 1e-10 || normalForce <= 0) return 0;
            double stress = (3.0 * normalForce) / (2.0 * Mathd.PI * contactRadius * contactRadius);
            if (double.IsNaN(stress) || double.IsInfinity(stress)) return 0;
            return stress;
        }

        public ContactForce SolveContact(
            ContactPoint contactPoint,
            ContactGeometry geometry,
            ContactMaterial material,
            MaterialProperties mat1,
            MaterialProperties mat2,
            double deltaTime)
        {
            ContactForce force = new ContactForce();

            double penetration = Mathd.Clamp(contactPoint.penetrationDepth, 0, maxPenetrationDepth);
            if (penetration <= 0) return force;

            Vector3d normal = contactPoint.normal;
            if (normal.SqrMagnitude < 1e-30) normal = Vector3d.Up;
            normal = normal.Normalized;

            double normalForceMag = CalculateHertzNormalForce(penetration, geometry, material);

            if (includePlasticity)
            {
                double contactRadius = CalculateContactRadius(penetration, geometry);
                double maxStress = CalculateMaximumNormalStress(normalForceMag, contactRadius);
                double h1 = mat1 != null ? mat1.hardness : 2e9;
                double h2 = mat2 != null ? mat2.hardness : 1e8;
                double minHardness = Mathd.Min(h1, h2);

                if (maxStress > 1.5 * minHardness && minHardness > 0)
                {
                    double equivR = Mathd.Max(geometry.equivalentRadius, 1e-6);
                    double equivE = Mathd.Clamp(material.equivalentModulus, 1e6, 1e13);
                    double maxPen = Mathd.Pow(
                        (2.25 * minHardness * minHardness * Mathd.PI * equivR) / (equivE * equivE),
                        1.0 / 3.0);
                    maxPen = Mathd.Min(maxPen, maxPenetrationDepth);
                    normalForceMag = CalculateHertzNormalForce(maxPen, geometry, material);
                }
            }

            force.normalForce = normal * normalForceMag;

            if (includeDamping)
            {
                double normalRelVel = Vector3d.Dot(contactPoint.relativeVelocity, normal);
                if (normalRelVel < 0)
                {
                    double d1 = mat1 != null ? mat1.normalDamping : 1e4;
                    double d2 = mat2 != null ? mat2.normalDamping : 1e4;
                    double dampingForce = Mathd.Abs(normalRelVel) * (d1 + d2) * 0.5;
                    dampingForce = Mathd.Min(dampingForce, normalForceMag * 0.5);
                    force.normalForce -= normal * dampingForce;
                }
            }

            double clampedNormalMag = force.normalForce.Magnitude;
            if (clampedNormalMag > maxNormalForce)
            {
                force.normalForce = force.normalForce.Normalized * maxNormalForce;
                clampedNormalMag = maxNormalForce;
            }

            if (useMindlinTheory)
            {
                force.tangentialForce = CalculateMindlinTangentialForce(
                    contactPoint, geometry, material, mat1, mat2, clampedNormalMag, deltaTime);
            }
            else
            {
                force.tangentialForce = CalculateCoulombFriction(
                    contactPoint, mat1, mat2, clampedNormalMag);
            }

            double tanMag = force.tangentialForce.Magnitude;
            if (tanMag > maxTangentialForce)
            {
                force.tangentialForce = force.tangentialForce.Normalized * maxTangentialForce;
                tanMag = maxTangentialForce;
            }

            force.frictionForce = tanMag;

            double contactRadius2 = CalculateContactRadius(penetration, geometry);
            force.normalStress = CalculateMaximumNormalStress(Mathd.Abs(clampedNormalMag), contactRadius2);
            double contactArea = CalculateContactArea(contactRadius2);
            force.shearStress = contactArea > 1e-30 ? tanMag / contactArea : 0;
            force.totalForce = force.normalForce + force.tangentialForce;

            double totalMag = force.totalForce.Magnitude;
            double forceLimit = maxNormalForce + maxTangentialForce;
            if (totalMag > forceLimit)
            {
                force.totalForce = force.totalForce.Normalized * forceLimit;
            }

            return force;
        }

        private Vector3d CalculateMindlinTangentialForce(
            ContactPoint contactPoint,
            ContactGeometry geometry,
            ContactMaterial material,
            MaterialProperties mat1,
            MaterialProperties mat2,
            double normalForce,
            double deltaTime)
        {
            if (normalForce <= 0) return Vector3d.Zero;

            Vector3d normal = contactPoint.normal;
            if (normal.SqrMagnitude < 1e-30) normal = Vector3d.Up;
            normal = normal.Normalized;

            Vector3d tangentialVel = contactPoint.tangentialVelocity;
            double tangentialSpeed = tangentialVel.Magnitude;

            if (tangentialSpeed < 1e-15) return Vector3d.Zero;

            Vector3d tangentialDir = tangentialVel.Normalized;
            double contactRadius = CalculateContactRadius(Mathd.Clamp(contactPoint.penetrationDepth, 0, maxPenetrationDepth), geometry);
            double contactArea = CalculateContactArea(contactRadius);

            double sf1 = mat1 != null ? mat1.staticFriction : 0.6;
            double sf2 = mat2 != null ? mat2.staticFriction : 0.5;
            double df1 = mat1 != null ? mat1.dynamicFriction : 0.4;
            double df2 = mat2 != null ? mat2.dynamicFriction : 0.3;

            double frictionCoeff = tangentialSpeed < 0.001
                ? (sf1 + sf2) * 0.5
                : (df1 + df2) * 0.5;

            double maxFrictionForce = frictionCoeff * normalForce;

            double relVelMag = Mathd.Max(contactPoint.relativeVelocity.Magnitude, 1e-15);
            double creep = tangentialSpeed / relVelMag;
            double creepRatio = Mathd.Clamp(creep / 0.01, 0, 1);

            double equivG = Mathd.Clamp(material.equivalentShearModulus, 1e6, 1e12);
            double tangentialStiffness = 8.0 * equivG * Mathd.Max(contactRadius, 1e-6);
            double elasticForce = tangentialStiffness * Mathd.Clamp(contactPoint.penetrationDepth, 0, maxPenetrationDepth) * creepRatio;

            double tangentialForceMag = Mathd.Min(elasticForce, maxFrictionForce);

            double td1 = mat1 != null ? mat1.tangentialDamping : 1e3;
            double td2 = mat2 != null ? mat2.tangentialDamping : 1e3;
            double dampingForce = tangentialSpeed * (td1 + td2) * 0.5 * contactArea;
            tangentialForceMag += dampingForce;
            tangentialForceMag = Mathd.Min(tangentialForceMag, maxFrictionForce);
            tangentialForceMag = Mathd.Min(tangentialForceMag, maxTangentialForce);

            return -tangentialDir * tangentialForceMag;
        }

        private Vector3d CalculateCoulombFriction(
            ContactPoint contactPoint,
            MaterialProperties mat1,
            MaterialProperties mat2,
            double normalForce)
        {
            if (normalForce <= 0) return Vector3d.Zero;

            Vector3d normal = contactPoint.normal;
            if (normal.SqrMagnitude < 1e-30) normal = Vector3d.Up;
            normal = normal.Normalized;

            Vector3d tangentialVel = contactPoint.tangentialVelocity;
            double tangentialSpeed = tangentialVel.Magnitude;

            if (tangentialSpeed < 1e-15) return Vector3d.Zero;

            Vector3d tangentialDir = tangentialVel.Normalized;

            double sf1 = mat1 != null ? mat1.staticFriction : 0.6;
            double sf2 = mat2 != null ? mat2.staticFriction : 0.5;
            double df1 = mat1 != null ? mat1.dynamicFriction : 0.4;
            double df2 = mat2 != null ? mat2.dynamicFriction : 0.3;

            double frictionCoeff = tangentialSpeed < 0.001
                ? (sf1 + sf2) * 0.5
                : (df1 + df2) * 0.5;

            double frictionForce = Mathd.Min(frictionCoeff * normalForce, maxTangentialForce);

            return -tangentialDir * frictionForce;
        }

        public Vector3d CalculateRollingResistance(
            Vector3d angularVelocity1,
            Vector3d angularVelocity2,
            double radius1,
            double radius2,
            double normalForce,
            double rollingResistanceCoeff)
        {
            if (normalForce <= 0) return Vector3d.Zero;

            Vector3d relativeSpin = angularVelocity1 - angularVelocity2;
            if (relativeSpin.SqrMagnitude < 1e-30) return Vector3d.Zero;

            double maxR = Mathd.Max(radius1, radius2);
            if (maxR <= 0) maxR = 0.01;

            Vector3d resistance = -relativeSpin.Normalized *
                Mathd.Min(normalForce, maxNormalForce) * Mathd.Clamp(rollingResistanceCoeff, 0, 1) * maxR;

            return ClampVec(resistance, maxTangentialForce);
        }

        private Vector3d ClampVec(Vector3d v, double maxMag)
        {
            double m = v.Magnitude;
            if (m > maxMag && m > 1e-15) return v.Normalized * maxMag;
            if (double.IsNaN(v.x) || double.IsNaN(v.y) || double.IsNaN(v.z) ||
                double.IsInfinity(v.x) || double.IsInfinity(v.y) || double.IsInfinity(v.z))
                return Vector3d.Zero;
            return v;
        }

        public double CalculateVonMisesStress(double normalStress, double shearStress)
        {
            double s = Mathd.Sqrt(normalStress * normalStress + 3.0 * shearStress * shearStress);
            if (double.IsNaN(s) || double.IsInfinity(s)) return 0;
            return s;
        }

        public bool CheckYield(double vonMisesStress, double yieldStrength)
        {
            if (yieldStrength <= 0) return false;
            return vonMisesStress > yieldStrength;
        }

        public double CalculateWearVolume(
            double normalForce,
            double slidingDistance,
            double hardness,
            double wearCoefficient = 1e-6)
        {
            if (hardness <= 0 || normalForce <= 0 || slidingDistance <= 0) return 0;
            double vol = Mathd.Clamp(wearCoefficient, 1e-12, 1e-3) * normalForce * slidingDistance / hardness;
            if (double.IsNaN(vol) || double.IsInfinity(vol)) return 0;
            return vol;
        }
    }
}
