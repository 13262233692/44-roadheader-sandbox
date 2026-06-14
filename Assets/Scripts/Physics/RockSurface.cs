using System;
using System.Collections.Generic;
using RoadheaderSandbox.Core.Math;
using UnityEngine;

namespace RoadheaderSandbox.Physics
{
    [Serializable]
    public class RockSurface : MonoBehaviour
    {
        [Header("几何参数")]
        [Tooltip("岩石断面宽度 (m)")]
        public double width = 6.0;

        [Tooltip("岩石断面高度 (m)")]
        public double height = 4.0;

        [Tooltip("岩石深度 (m)")]
        public double depth = 10.0;

        [Tooltip("网格分辨率 (点数/m)")]
        public double resolution = 10.0;

        [Header("表面粗糙度")]
        [Tooltip("粗糙度振幅 (m)")]
        public double roughnessAmplitude = 0.02;

        [Tooltip("粗糙度频率")]
        public double roughnessFrequency = 5.0;

        [Header("材料属性")]
        public MaterialProperties material = null;

        [Header("已开采区域")]
        public List<Bounds> excavatedRegions = new List<Bounds>();

        private double[,] _heightField;
        private int _gridWidth;
        private int _gridHeight;
        private double _cellSize;
        private bool _initialized;

        public event Action<Vector3d, double> OnExcavated;

        private void Awake()
        {
            if (material == null)
            {
                material = MaterialProperties.Rock();
            }
            InitializeHeightField();
        }

        public void InitializeHeightField()
        {
            _cellSize = 1.0 / resolution;
            _gridWidth = (int)Math.Ceiling(width / _cellSize) + 1;
            _gridHeight = (int)Math.Ceiling(height / _cellSize) + 1;
            _heightField = new double[_gridWidth, _gridHeight];

            for (int i = 0; i < _gridWidth; i++)
            {
                for (int j = 0; j < _gridHeight; j++)
                {
                    double x = i * _cellSize - width * 0.5;
                    double y = j * _cellSize - height * 0.5;
                    _heightField[i, j] = CalculateInitialHeight(x, y);
                }
            }

            _initialized = true;
        }

        private double CalculateInitialHeight(double x, double y)
        {
            double baseHeight = 0;

            double noise1 = Mathd.Sin(x * roughnessFrequency) * Mathd.Cos(y * roughnessFrequency);
            double noise2 = Mathd.Sin(x * roughnessFrequency * 2.5 + 1.3) * Mathd.Cos(y * roughnessFrequency * 2.5 + 0.7);
            double noise3 = Mathd.Sin(x * roughnessFrequency * 0.7 + 2.1) * Mathd.Cos(y * roughnessFrequency * 0.7 + 1.1);

            double roughness = roughnessAmplitude * (noise1 * 0.5 + noise2 * 0.3 + noise3 * 0.2);

            double boundaryFade = 1.0;
            double edgeX = Mathd.Abs(x) - width * 0.5 + 0.5;
            double edgeY = Mathd.Abs(y) - height * 0.5 + 0.5;
            if (edgeX > 0) boundaryFade *= Mathd.Exp(-edgeX * edgeX * 4.0);
            if (edgeY > 0) boundaryFade *= Mathd.Exp(-edgeY * edgeY * 4.0);

            return baseHeight + roughness * boundaryFade;
        }

        public double GetHeight(double x, double y)
        {
            if (!_initialized) InitializeHeightField();

            double localX = x + width * 0.5;
            double localY = y + height * 0.5;

            int i0 = (int)Math.Floor(localX / _cellSize);
            int j0 = (int)Math.Floor(localY / _cellSize);
            int i1 = i0 + 1;
            int j1 = j0 + 1;

            i0 = Mathd.Clamp(i0, 0, _gridWidth - 1);
            j0 = Mathd.Clamp(j0, 0, _gridHeight - 1);
            i1 = Mathd.Clamp(i1, 0, _gridWidth - 1);
            j1 = Mathd.Clamp(j1, 0, _gridHeight - 1);

            double fx = (localX - i0 * _cellSize) / _cellSize;
            double fy = (localY - j0 * _cellSize) / _cellSize;

            fx = Mathd.Clamp01(fx);
            fy = Mathd.Clamp01(fy);

            double h00 = _heightField[i0, j0];
            double h10 = _heightField[i1, j0];
            double h01 = _heightField[i0, j1];
            double h11 = _heightField[i1, j1];

            double h0 = Mathd.Lerp(h00, h10, fx);
            double h1 = Mathd.Lerp(h01, h11, fx);
            double h = Mathd.Lerp(h0, h1, fy);

            h = Mathd.Max(h, -depth);

            foreach (var region in excavatedRegions)
            {
                if (x >= region.min.x && x <= region.max.x &&
                    y >= region.min.y && y <= region.max.y)
                {
                    double regionDepth = -region.max.z;
                    if (h < regionDepth)
                        h = regionDepth;
                }
            }

            return h;
        }

        public Vector3d GetNormal(double x, double y)
        {
            if (!_initialized) InitializeHeightField();

            double eps = _cellSize * 0.5;
            double hL = GetHeight(x - eps, y);
            double hR = GetHeight(x + eps, y);
            double hD = GetHeight(x, y - eps);
            double hU = GetHeight(x, y + eps);

            Vector3d tangentX = new Vector3d(2.0 * eps, 0, hR - hL);
            Vector3d tangentY = new Vector3d(0, 2.0 * eps, hU - hD);

            return Vector3d.Cross(tangentX, tangentY).Normalized;
        }

        public double GetSignedDistance(Vector3d point)
        {
            double surfaceZ = GetHeight(point.x, point.y);
            double distance = point.z - surfaceZ;

            foreach (var region in excavatedRegions)
            {
                Vector3d closest = new Vector3d(
                    Mathd.Clamp(point.x, region.min.x, region.max.x),
                    Mathd.Clamp(point.y, region.min.y, region.max.y),
                    Mathd.Clamp(point.z, region.min.z, region.max.z)
                );

                if (region.Contains(closest.ToVector3()))
                {
                    double distToRegion = (point - closest).Magnitude;
                    if (point.z < region.max.z)
                        distToRegion = -distToRegion;
                    distance = Mathd.Max(distance, distToRegion);
                }
            }

            return distance;
        }

        public Vector3d GetGradient(Vector3d point)
        {
            double eps = 0.001;
            double dx = GetSignedDistance(point + new Vector3d(eps, 0, 0)) - GetSignedDistance(point - new Vector3d(eps, 0, 0));
            double dy = GetSignedDistance(point + new Vector3d(0, eps, 0)) - GetSignedDistance(point - new Vector3d(0, eps, 0));
            double dz = GetSignedDistance(point + new Vector3d(0, 0, eps)) - GetSignedDistance(point - new Vector3d(0, 0, eps));

            return new Vector3d(dx, dy, dz) / (2.0 * eps);
        }

        public ContactPoint CheckCollision(Vector3d point, double radius, Vector3d velocity)
        {
            ContactPoint contact = new ContactPoint();

            double distance = GetSignedDistance(point);
            double penetration = radius - distance;

            if (penetration > 0)
            {
                contact.position = point;
                contact.normal = GetGradient(point).Normalized;
                contact.penetrationDepth = penetration;
                contact.relativeVelocity = velocity;
                contact.normalVelocity = contact.normal * Vector3d.Dot(velocity, contact.normal);
                contact.tangentialVelocity = velocity - contact.normalVelocity;

                double contactRadius = Mathd.Sqrt(2.0 * radius * penetration - penetration * penetration);
                contact.contactRadius = contactRadius;
                contact.contactArea = Mathd.PI * contactRadius * contactRadius;
            }

            return contact;
        }

        public void Excavate(Vector3d position, double radius, double depth)
        {
            Bounds region = new Bounds(
                position.ToVector3(),
                new Vector3((float)(radius * 2), (float)(radius * 2), (float)depth)
            );
            excavatedRegions.Add(region);

            OnExcavated?.Invoke(position, depth);
        }

        public void ExcavateSphere(Vector3d center, double radius)
        {
            int iMin = Math.Max(0, (int)Math.Floor((center.x - radius + width * 0.5) / _cellSize));
            int iMax = Math.Min(_gridWidth - 1, (int)Math.Ceiling((center.x + radius + width * 0.5) / _cellSize));
            int jMin = Math.Max(0, (int)Math.Floor((center.y - radius + height * 0.5) / _cellSize));
            int jMax = Math.Min(_gridHeight - 1, (int)Math.Ceiling((center.y + radius + height * 0.5) / _cellSize));

            for (int i = iMin; i <= iMax; i++)
            {
                for (int j = jMin; j <= jMax; j++)
                {
                    double x = i * _cellSize - width * 0.5;
                    double y = j * _cellSize - height * 0.5;

                    double dx = x - center.x;
                    double dy = y - center.y;
                    double distSq = dx * dx + dy * dy;

                    if (distSq < radius * radius)
                    {
                        double zOffset = Mathd.Sqrt(radius * radius - distSq);
                        double newHeight = center.z - zOffset;
                        if (newHeight > _heightField[i, j])
                        {
                            _heightField[i, j] = Mathd.Max(newHeight, _heightField[i, j]);
                        }
                    }
                }
            }

            OnExcavated?.Invoke(center, radius);
        }

        public List<Vector3d> GetSurfacePoints(int subsample = 1)
        {
            if (!_initialized) InitializeHeightField();

            List<Vector3d> points = new List<Vector3d>();
            for (int i = 0; i < _gridWidth; i += subsample)
            {
                for (int j = 0; j < _gridHeight; j += subsample)
                {
                    double x = i * _cellSize - width * 0.5;
                    double y = j * _cellSize - height * 0.5;
                    double z = _heightField[i, j];
                    points.Add(new Vector3d(x, y, z));
                }
            }
            return points;
        }

        private void OnDrawGizmos()
        {
            if (!_initialized) return;

            Gizmos.color = new Color(0.6f, 0.4f, 0.2f, 0.8f);
            int subsample = Math.Max(1, (int)resolution / 2);

            for (int i = 0; i < _gridWidth - subsample; i += subsample)
            {
                for (int j = 0; j < _gridHeight - subsample; j += subsample)
                {
                    double x0 = i * _cellSize - width * 0.5;
                    double y0 = j * _cellSize - height * 0.5;
                    double x1 = (i + subsample) * _cellSize - width * 0.5;
                    double y1 = (j + subsample) * _cellSize - height * 0.5;

                    Vector3 p00 = new Vector3((float)x0, (float)y0, (float)_heightField[i, j]);
                    Vector3 p10 = new Vector3((float)x1, (float)y0, (float)_heightField[i + subsample, j]);
                    Vector3 p01 = new Vector3((float)x0, (float)y1, (float)_heightField[i, j + subsample]);
                    Vector3 p11 = new Vector3((float)x1, (float)y1, (float)_heightField[i + subsample, j + subsample]);

                    Gizmos.DrawLine(p00, p10);
                    Gizmos.DrawLine(p00, p01);
                }
            }

            Gizmos.color = Color.red;
            foreach (var region in excavatedRegions)
            {
                Gizmos.DrawWireCube(region.center, region.size);
            }
        }
    }
}
