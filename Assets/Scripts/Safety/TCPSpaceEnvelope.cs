using System;
using System.Collections.Generic;
using RoadheaderSandbox.Core.Math;
using RoadheaderSandbox.Physics;
using UnityEngine;

namespace RoadheaderSandbox.Safety
{
    [Serializable]
    public struct EnvelopePoint
    {
        public Vector3d position;
        public Quaterniond rotation;
        public Vector3d linearVelocity;
        public Vector3d angularVelocity;
        public double kineticEnergy;
        public double momentum;
        public double predictedTime;
        public double hardnessOnPath;
        public double predictedForce;
        public int sampleIndex;

        public EnvelopePoint(int idx)
        {
            sampleIndex = idx;
            position = Vector3d.Zero;
            rotation = Quaterniond.Identity;
            linearVelocity = Vector3d.Zero;
            angularVelocity = Vector3d.Zero;
            kineticEnergy = 0;
            momentum = 0;
            predictedTime = 0;
            hardnessOnPath = 0;
            predictedForce = 0;
        }
    }

    [Serializable]
    public struct HardnessBoundary
    {
        public Vector3d center;
        public double radius;
        public double compressiveStrength;
        public string materialTag;
        public int boundaryId;

        public HardnessBoundary(int id, Vector3d c, double r, double str, string tag)
        {
            boundaryId = id;
            center = c;
            radius = r;
            compressiveStrength = str;
            materialTag = tag;
        }

        public double DistanceTo(Vector3d p)
        {
            return Mathd.Max(0, (p - center).Magnitude - radius);
        }

        public bool Contains(Vector3d p, double safetyMargin = 0)
        {
            return (p - center).SqrMagnitude <= (radius + safetyMargin) * (radius + safetyMargin);
        }
    }

    [Serializable]
    public struct CollisionPrediction
    {
        public bool willCollide;
        public double collisionTime;
        public Vector3d collisionPoint;
        public double impactEnergy;
        public double impactForce;
        public double targetHardness;
        public double overloadRatio;
        public int boundaryId;
        public int envelopeSampleIndex;
        public double requiredSpeedReductionRatio;

        public static CollisionPrediction None => new CollisionPrediction
        {
            willCollide = false,
            collisionTime = double.MaxValue,
            overloadRatio = 1.0,
            requiredSpeedReductionRatio = 1.0
        };
    }

    [Serializable]
    public class TCPSpaceEnvelope : MonoBehaviour
    {
        [Header("时间推演")]
        [Tooltip("前馈预判时长 (s)")]
        public double predictionHorizon = 0.5;

        [Tooltip("包络线采样点数 (越大越精细但更慢)")]
        public int sampleCount = 50;

        [Tooltip("单步预测最大距离 (m)")]
        public double maxStepPrediction = 0.1;

        [Header("动能评估")]
        [Tooltip("截割头等效质量 (kg)")]
        public double headEquivalentMass = 2500.0;

        [Tooltip("截割头转动惯量 (kg·m²)")]
        public double headMomentOfInertia = 120.0;

        [Tooltip("允许最大冲击能量 (J) - 超限触发限速")]
        public double maxImpactEnergyThreshold = 50000.0;

        [Tooltip("单齿允许最大瞬时切削力 (N)")]
        public double allowablePickForce = 30000.0;

        [Header("硬度边界（铁矿/夹矸等高危区）")]
        public List<HardnessBoundary> hardnessBoundaries = new List<HardnessBoundary>();

        [Header("引用")]
        public RockSurface rockSurface;

        [Header("诊断")]
        public int computedSamples;
        public double computeTimeUs;
        public double maxKineticEnergyOnPath;
        public double maxHardnessOnPath;
        public CollisionPrediction latestPrediction;

        public event Action<EnvelopePoint[]> OnEnvelopeUpdated;
        public event Action<CollisionPrediction> OnCollisionPredicted;

        private EnvelopePoint[] _envelope;
        private Vector3d _lastTcpPos;
        private Quaterniond _lastTcpRot;
        private double _lastUpdateTime;

        public EnvelopePoint[] CurrentEnvelope => _envelope;

        private void Awake()
        {
            _envelope = new EnvelopePoint[sampleCount];
            for (int i = 0; i < sampleCount; i++)
                _envelope[i] = new EnvelopePoint(i);

            if (rockSurface == null)
                rockSurface = FindObjectOfType<RockSurface>();

            InitializeDefaultBoundaries();
        }

        private void InitializeDefaultBoundaries()
        {
            hardnessBoundaries.Clear();

            hardnessBoundaries.Add(new HardnessBoundary(
                1,
                new Vector3d(1.5, 0.5, -4.0),
                1.2,
                350e6,
                "HardIronCore"
            ));

            hardnessBoundaries.Add(new HardnessBoundary(
                2,
                new Vector3d(-2.0, -0.3, -6.5),
                0.9,
                280e6,
                "GangueClamp"
            ));

            hardnessBoundaries.Add(new HardnessBoundary(
                3,
                new Vector3d(0.5, 1.2, -8.0),
                1.5,
                420e6,
                "UltraHardQuartz"
            ));
        }

        public void AddBoundary(Vector3d center, double radius, double strength, string tag)
        {
            int id = hardnessBoundaries.Count + 1;
            hardnessBoundaries.Add(new HardnessBoundary(id, center, radius, strength, tag));
        }

        public unsafe CollisionPrediction Compute(Vector3d tcpPosition, Quaterniond tcpRotation,
                                                   Vector3d linearVel, Vector3d angularVel,
                                                   double dt = 0.01)
        {
            double t0 = (double)(DateTime.Now.Ticks / 10.0);

            double horizon = Mathd.Clamp(predictionHorizon, 0.02, 5.0);
            int samples = Mathd.Clamp(sampleCount, 4, 200);
            if (_envelope == null || _envelope.Length != samples)
            {
                _envelope = new EnvelopePoint[samples];
                for (int i = 0; i < samples; i++) _envelope[i] = new EnvelopePoint(i);
            }

            double invSamples = 1.0 / Mathd.Max(1, samples - 1);
            double maxKE = 0;
            double maxH = 0;

            Vector3d pos = tcpPosition;
            Quaterniond rot = tcpRotation;
            double linearSpeed = linearVel.Magnitude;
            double angularSpeed = angularVel.Magnitude;
            Vector3d linDir = linearSpeed > 1e-15 ? linearVel.Normalized : Vector3d.Forward;
            Vector3d angDir = angularSpeed > 1e-15 ? angularVel.Normalized : Vector3d.Forward;

            fixed (EnvelopePoint* env = _envelope)
            {
                for (int i = 0; i < samples; i++)
                {
                    double t = i * invSamples * horizon;

                    Vector3d step = linDir * (linearSpeed * t);
                    if (step.Magnitude > maxStepPrediction * samples * 0.25)
                        step = step.Normalized * (maxStepPrediction * samples * 0.25);

                    env[i].sampleIndex = i;
                    env[i].position = pos + step;
                    env[i].rotation = Quaterniond.Slerp(rot, rot * Quaterniond.AxisAngle(angDir, angularSpeed * t), 1.0);
                    env[i].linearVelocity = linearVel;
                    env[i].angularVelocity = angularVel;
                    env[i].predictedTime = t;

                    double transKE = 0.5 * headEquivalentMass * linearSpeed * linearSpeed;
                    double rotKE = 0.5 * headMomentOfInertia * angularSpeed * angularSpeed;
                    env[i].kineticEnergy = transKE + rotKE;
                    env[i].momentum = headEquivalentMass * linearSpeed + headMomentOfInertia * angularSpeed;

                    env[i].hardnessOnPath = SampleHardness(env[i].position);

                    double projectedArea = 0.003;
                    double penetrationEst = Mathd.Clamp(linearSpeed * dt * 5.0, 0, 0.01);
                    env[i].predictedForce = env[i].hardnessOnPath * projectedArea *
                                            Mathd.Sqrt(Mathd.Max(0.0001, penetrationEst / 0.01));

                    if (env[i].kineticEnergy > maxKE) maxKE = env[i].kineticEnergy;
                    if (env[i].hardnessOnPath > maxH) maxH = env[i].hardnessOnPath;
                }
            }

            CollisionPrediction prediction = DetectCollision(_envelope, horizon, samples);

            computedSamples = samples;
            computeTimeUs = (double)(DateTime.Now.Ticks / 10.0) - t0;
            maxKineticEnergyOnPath = maxKE;
            maxHardnessOnPath = maxH;
            latestPrediction = prediction;
            _lastTcpPos = tcpPosition;
            _lastTcpRot = tcpRotation;
            _lastUpdateTime = Time.time;

            OnEnvelopeUpdated?.Invoke(_envelope);
            if (prediction.willCollide)
                OnCollisionPredicted?.Invoke(prediction);

            return prediction;
        }

        public double SampleHardness(Vector3d worldPos)
        {
            double baseHardness = 50e6;

            if (rockSurface != null && rockSurface.material != null)
                baseHardness = rockSurface.material.compressiveStrength;

            foreach (var b in hardnessBoundaries)
            {
                double d = (worldPos - b.center).Magnitude;
                if (d < b.radius)
                {
                    double fade = Mathd.Exp(-d * d / Mathd.Max(0.01, b.radius * b.radius * 0.25));
                    baseHardness = Mathd.Max(baseHardness, b.compressiveStrength * fade);
                }
            }

            return baseHardness;
        }

        public unsafe CollisionPrediction DetectCollision(EnvelopePoint[] env, double horizon, int count)
        {
            fixed (EnvelopePoint* e = env)
            {
                for (int i = 0; i < count; i++)
                {
                    double hardness = e[i].hardnessOnPath;
                    double impactEnergy = e[i].kineticEnergy;
                    double impactForce = e[i].predictedForce;
                    double criticalForce = allowablePickForce * 42;

                    if (hardness > 200e6 || impactEnergy > maxImpactEnergyThreshold || impactForce > criticalForce * 0.8)
                    {
                        int bId = -1;
                        for (int b = 0; b < hardnessBoundaries.Count; b++)
                        {
                            if (hardnessBoundaries[b].Contains(e[i].position, 0.2))
                            {
                                bId = hardnessBoundaries[b].boundaryId;
                                break;
                            }
                        }

                        double overloadRatio = Mathd.Max(
                            hardness / 200e6,
                            Mathd.Max(
                                impactEnergy / Mathd.Max(1e-6, maxImpactEnergyThreshold),
                                impactForce / Mathd.Max(1e-6, criticalForce)
                            ));

                        double safeEnergy = maxImpactEnergyThreshold * 0.5;
                        double ratio = overloadRatio > 1.0001
                            ? Mathd.Sqrt(Mathd.Max(0, safeEnergy / Mathd.Max(1.0, impactEnergy)))
                            : 1.0;
                        ratio = Mathd.Clamp(ratio, 0.05, 1.0);

                        return new CollisionPrediction
                        {
                            willCollide = true,
                            collisionTime = e[i].predictedTime,
                            collisionPoint = e[i].position,
                            impactEnergy = impactEnergy,
                            impactForce = impactForce,
                            targetHardness = hardness,
                            overloadRatio = overloadRatio,
                            boundaryId = bId,
                            envelopeSampleIndex = i,
                            requiredSpeedReductionRatio = ratio
                        };
                    }
                }
            }

            return CollisionPrediction.None;
        }

        public HardnessBoundary FindBoundary(int id)
        {
            for (int i = 0; i < hardnessBoundaries.Count; i++)
                if (hardnessBoundaries[i].boundaryId == id)
                    return hardnessBoundaries[i];
            return default;
        }

        public bool IsInHardRegion(Vector3d p, double threshold = 200e6)
        {
            return SampleHardness(p) >= threshold;
        }

        private void OnDrawGizmos()
        {
            if (_envelope == null) return;

            Color c0 = Gizmos.color;

            for (int i = 0; i < _envelope.Length; i++)
            {
                double t = (double)i / Mathd.Max(1, _envelope.Length - 1);
                double ratio = Mathd.Clamp01(_envelope[i].hardnessOnPath / 400e6);
                Gizmos.color = Color.Lerp(new Color(0, 0.8f, 0, 0.4f), new Color(1, 0.1f, 0, 0.9f), (float)ratio);
                Gizmos.DrawSphere(_envelope[i].position.ToVector3(), 0.03f + (float)(ratio * 0.06));
            }

            Gizmos.color = new Color(1, 0.85f, 0, 0.4f);
            for (int i = 1; i < _envelope.Length; i++)
            {
                Gizmos.DrawLine(_envelope[i - 1].position.ToVector3(), _envelope[i].position.ToVector3());
            }

            foreach (var b in hardnessBoundaries)
            {
                float ratio = (float)Mathd.Clamp01(b.compressiveStrength / 400e6);
                Gizmos.color = Color.Lerp(new Color(1, 0.5f, 0, 0.25f), new Color(1, 0, 0.2f, 0.5f), ratio);
                Gizmos.DrawWireSphere(b.center.ToVector3(), (float)b.radius);
                Gizmos.color = Color.Lerp(new Color(1, 0.7f, 0.1f, 0.08f), new Color(1, 0.1f, 0.2f, 0.18f), ratio);
                Gizmos.DrawSphere(b.center.ToVector3(), (float)b.radius);
            }

            Gizmos.color = c0;
        }
    }
}
