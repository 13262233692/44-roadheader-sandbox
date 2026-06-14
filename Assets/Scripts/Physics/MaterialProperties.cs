using System;
using UnityEngine;

namespace RoadheaderSandbox.Physics
{
    [Serializable]
    public class MaterialProperties
    {
        [Header("弹性属性")]
        [Tooltip("杨氏模量 (Pa)")]
        public double youngsModulus = 70e9;

        [Tooltip("泊松比")]
        [Range(0, 0.5)]
        public double poissonsRatio = 0.3;

        [Header("塑性属性")]
        [Tooltip("屈服强度 (Pa)")]
        public double yieldStrength = 250e6;

        [Tooltip("硬度 (Pa)")]
        public double hardness = 2.5e9;

        [Header("摩擦属性")]
        [Tooltip("静摩擦系数")]
        public double staticFriction = 0.6;

        [Tooltip("动摩擦系数")]
        public double dynamicFriction = 0.4;

        [Header("阻尼属性")]
        [Tooltip("法向恢复系数")]
        [Range(0, 1)]
        public double restitution = 0.1;

        [Tooltip("法向阻尼系数")]
        public double normalDamping = 1e4;

        [Tooltip("切向阻尼系数")]
        public double tangentialDamping = 1e3;

        [Header("岩石属性")]
        [Tooltip("抗压强度 (Pa)")]
        public double compressiveStrength = 120e6;

        [Tooltip("抗拉强度 (Pa)")]
        public double tensileStrength = 8e6;

        [Tooltip("内聚力 (Pa)")]
        public double cohesion = 20e6;

        [Tooltip("内摩擦角 (度)")]
        public double frictionAngle = 30.0;

        [Header("密度")]
        [Tooltip("密度 (kg/m³)")]
        public double density = 7850.0;

        public double ShearModulus
        {
            get { return youngsModulus / (2.0 * (1.0 + poissonsRatio)); }
        }

        public double BulkModulus
        {
            get { return youngsModulus / (3.0 * (1.0 - 2.0 * poissonsRatio)); }
        }

        public static MaterialProperties Steel()
        {
            return new MaterialProperties
            {
                youngsModulus = 210e9,
                poissonsRatio = 0.3,
                yieldStrength = 350e6,
                hardness = 5.0e9,
                staticFriction = 0.5,
                dynamicFriction = 0.4,
                restitution = 0.2,
                normalDamping = 1e5,
                tangentialDamping = 1e4,
                compressiveStrength = 500e6,
                tensileStrength = 250e6,
                cohesion = 0,
                frictionAngle = 10.0,
                density = 7850.0
            };
        }

        public static MaterialProperties Coal()
        {
            return new MaterialProperties
            {
                youngsModulus = 15e9,
                poissonsRatio = 0.25,
                yieldStrength = 30e6,
                hardness = 0.5e9,
                staticFriction = 0.6,
                dynamicFriction = 0.45,
                restitution = 0.05,
                normalDamping = 5e3,
                tangentialDamping = 5e2,
                compressiveStrength = 40e6,
                tensileStrength = 3e6,
                cohesion = 15e6,
                frictionAngle = 25.0,
                density = 1350.0
            };
        }

        public static MaterialProperties Rock()
        {
            return new MaterialProperties
            {
                youngsModulus = 70e9,
                poissonsRatio = 0.28,
                yieldStrength = 100e6,
                hardness = 2.5e9,
                staticFriction = 0.7,
                dynamicFriction = 0.5,
                restitution = 0.08,
                normalDamping = 1e4,
                tangentialDamping = 1e3,
                compressiveStrength = 120e6,
                tensileStrength = 8e6,
                cohesion = 25e6,
                frictionAngle = 35.0,
                density = 2650.0
            };
        }

        public static MaterialProperties TungstenCarbide()
        {
            return new MaterialProperties
            {
                youngsModulus = 600e9,
                poissonsRatio = 0.22,
                yieldStrength = 800e6,
                hardness = 15.0e9,
                staticFriction = 0.3,
                dynamicFriction = 0.2,
                restitution = 0.15,
                normalDamping = 2e5,
                tangentialDamping = 2e4,
                compressiveStrength = 3000e6,
                tensileStrength = 500e6,
                cohesion = 0,
                frictionAngle = 5.0,
                density = 15600.0
            };
        }
    }
}
