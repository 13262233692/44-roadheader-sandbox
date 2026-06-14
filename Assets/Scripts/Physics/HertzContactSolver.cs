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
            material.youngsModulus1 = mat1.youngsModulus;
            material.youngsModulus2 = mat2.youngsModulus;
            material.poissonsRatio1 = mat1.poissonsRatio;
            material.poissonsRatio2 = mat2.poissonsRatio;
            material.shearModulus1 = mat1.ShearModulus;
            material.shearModulus2 = mat2.ShearModulus;

            material.equivalentModulus = 1.0 / (
                (1.0 - mat1.poissonsRatio * mat1.poissonsRatio) / mat1.youngsModulus +
                (1.0 - mat2.poissonsRatio * mat2.poissonsRatio) / mat2.youngsModulus
            );

            material.equivalentShearModulus = 1.0 / (
                (2.0 - mat1.poissonsRatio) / mat1.ShearModulus +
                (2.0 - mat2.poissonsRatio) / mat2.ShearModulus
            ) * 0.5;

            return material;
        }

        public ContactGeometry CalculateEquivalentGeometry(double radius1, double radius2, Vector3d normal)
        {
            ContactGeometry geometry = new ContactGeometry();
            geometry.radius1 = radius1;
            geometry.radius2 = radius2;
            geometry.normal = normal.Normalized;

            if (radius1 <= 0 || radius2 <= 0)
            {
                geometry.equivalentRadius = Mathd.Min(radius1, radius2);
                if (geometry.equivalentRadius <= 0) geometry.equivalentRadius = 0.001;
            }
            else
            {
                geometry.curvatureSum = 1.0 / radius1 + 1.0 / radius2;
                geometry.curvatureDifference = Mathd.Abs(1.0 / radius1 - 1.0 / radius2);
                geometry.equivalentRadius = 1.0 / geometry.curvatureSum;
            }

            return geometry;
        }

        public double CalculateHertzNormalForce(double penetrationDepth, ContactGeometry geometry, ContactMaterial material)
        {
            if (penetrationDepth <= 0) return 0;

            double delta = Mathd.Max(penetrationDepth, 1e-15);
            double force = (4.0 / 3.0) * material.equivalentModulus *
                          Mathd.Sqrt(geometry.equivalentRadius) *
                          Mathd.Pow(delta, 1.5);

            if (double.IsNaN(force) || double.IsInfinity(force))
            {
                force = 0;
            }

            return force;
        }

        public double CalculateContactRadius(double penetrationDepth, ContactGeometry geometry)
        {
            if (penetrationDepth <= 0) return 0;
            return Mathd.Sqrt(geometry.equivalentRadius * penetrationDepth);
        }

        public double CalculateContactArea(double contactRadius)
        {
            return Mathd.PI * contactRadius * contactRadius;
        }

        public double CalculateMaximumNormalStress(double normalForce, double contactRadius)
        {
            if (contactRadius <= 0) return 0;
            return (3.0 * normalForce) / (2.0 * Mathd.PI * contactRadius * contactRadius);
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

            if (contactPoint.penetrationDepth <= 0)
            {
                return force;
            }

            double normalForceMag = CalculateHertzNormalForce(
                contactPoint.penetrationDepth, geometry, material);

            if (includePlasticity)
            {
                double contactRadius = CalculateContactRadius(contactPoint.penetrationDepth, geometry);
                double maxStress = CalculateMaximumNormalStress(normalForceMag, contactRadius);
                double minHardness = Mathd.Min(mat1.hardness, mat2.hardness);

                if (maxStress > 1.5 * minHardness)
                {
                    double maxPenetration = Mathd.Pow(
                        (2.25 * minHardness * minHardness * Mathd.PI * geometry.equivalentRadius) /
                        (material.equivalentModulus * material.equivalentModulus),
                        1.0 / 3.0);

                    normalForceMag = CalculateHertzNormalForce(maxPenetration, geometry, material);
                }
            }

            Vector3d normal = contactPoint.normal.Normalized;
            force.normalForce = normal * normalForceMag;

            if (includeDamping)
            {
                double normalRelVel = Vector3d.Dot(contactPoint.relativeVelocity, normal);
                if (normalRelVel < 0)
                {
                    double dampingForce = Mathd.Abs(normalRelVel) *
                        (mat1.normalDamping + mat2.normalDamping) * 0.5;
                    force.normalForce -= normal * dampingForce;
                }
            }

            if (useMindlinTheory)
            {
                force.tangentialForce = CalculateMindlinTangentialForce(
                    contactPoint, geometry, material, mat1, mat2, normalForceMag, deltaTime);
            }
            else
            {
                force.tangentialForce = CalculateCoulombFriction(
                    contactPoint, mat1, mat2, normalForceMag);
            }

            force.frictionForce = force.tangentialForce.Magnitude;
            force.normalStress = CalculateMaximumNormalStress(
                Mathd.Abs(normalForceMag),
                CalculateContactRadius(contactPoint.penetrationDepth, geometry));
            force.shearStress = force.frictionForce / CalculateContactArea(
                CalculateContactRadius(contactPoint.penetrationDepth, geometry));
            force.totalForce = force.normalForce + force.tangentialForce;

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
            Vector3d normal = contactPoint.normal.Normalized;
            Vector3d tangentialVel = contactPoint.tangentialVelocity;
            double tangentialSpeed = tangentialVel.Magnitude;

            if (tangentialSpeed < 1e-15) return Vector3d.Zero;

            Vector3d tangentialDir = tangentialVel.Normalized;
            double contactRadius = CalculateContactRadius(contactPoint.penetrationDepth, geometry);
            double contactArea = CalculateContactArea(contactRadius);

            double frictionCoeff = tangentialSpeed < 0.001
                ? (mat1.staticFriction + mat2.staticFriction) * 0.5
                : (mat1.dynamicFriction + mat2.dynamicFriction) * 0.5;

            double maxFrictionForce = frictionCoeff * normalForce;

            double creep = tangentialSpeed / Mathd.Max(contactPoint.relativeVelocity.Magnitude, 1e-15);
            double creepRatio = Mathd.Clamp(creep / 0.01, 0, 1);

            double tangentialStiffness = 8.0 * material.equivalentShearModulus * contactRadius;
            double elasticForce = tangentialStiffness * contactPoint.penetrationDepth * creepRatio;

            double tangentialForceMag = Mathd.Min(elasticForce, maxFrictionForce);

            double dampingForce = tangentialSpeed *
                (mat1.tangentialDamping + mat2.tangentialDamping) * 0.5 * contactArea;
            tangentialForceMag += dampingForce;
            tangentialForceMag = Mathd.Min(tangentialForceMag, maxFrictionForce);

            return -tangentialDir * tangentialForceMag;
        }

        private Vector3d CalculateCoulombFriction(
            ContactPoint contactPoint,
            MaterialProperties mat1,
            MaterialProperties mat2,
            double normalForce)
        {
            Vector3d normal = contactPoint.normal.Normalized;
            Vector3d tangentialVel = contactPoint.tangentialVelocity;
            double tangentialSpeed = tangentialVel.Magnitude;

            if (tangentialSpeed < 1e-15) return Vector3d.Zero;

            Vector3d tangentialDir = tangentialVel.Normalized;

            double frictionCoeff = tangentialSpeed < 0.001
                ? (mat1.staticFriction + mat2.staticFriction) * 0.5
                : (mat1.dynamicFriction + mat2.dynamicFriction) * 0.5;

            double frictionForce = frictionCoeff * normalForce;

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
            Vector3d rollingResistance = -relativeSpin.Normalized *
                normalForce * rollingResistanceCoeff *
                Mathd.Max(radius1, radius2);

            return rollingResistance;
        }

        public double CalculateVonMisesStress(double normalStress, double shearStress)
        {
            return Mathd.Sqrt(normalStress * normalStress + 3.0 * shearStress * shearStress);
        }

        public bool CheckYield(double vonMisesStress, double yieldStrength)
        {
            return vonMisesStress > yieldStrength;
        }

        public double CalculateWearVolume(
            double normalForce,
            double slidingDistance,
            double hardness,
            double wearCoefficient = 1e-6)
        {
            if (hardness <= 0) return 0;
            return wearCoefficient * normalForce * slidingDistance / hardness;
        }
    }
}
