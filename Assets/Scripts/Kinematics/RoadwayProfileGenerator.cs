using System;
using System.Collections.Generic;
using RoadheaderSandbox.Core.Math;
using UnityEngine;

namespace RoadheaderSandbox.Kinematics
{
    [Serializable]
    public class RoadwayProfileGenerator : MonoBehaviour
    {
        [Header("断面几何参数")]
        [Tooltip("巷道宽度 (m)")]
        public double roadwayWidth = 5.0;

        [Tooltip("巷道高度 (m)")]
        public double roadwayHeight = 3.5;

        [Tooltip("底部圆弧半径 (m)")]
        public double floorRadius = 0.5;

        [Tooltip("顶部圆弧半径 (m)")]
        public double roofRadius = 2.0;

        [Tooltip("侧帮倾斜角 (度)")]
        public double sideAngle = 5.0;

        [Tooltip("断面中心点 (机体坐标系)")]
        public Vector3 profileCenter = new Vector3(0, 1.75f, 0);

        [Header("截割路径参数")]
        [Tooltip("截割进给步距 (m)")]
        public double feedStep = 0.5;

        [Tooltip("最大进给深度 (m)")]
        public double maxFeedDepth = 10.0;

        [Tooltip("每段采样点数")]
        public int samplesPerSegment = 20;

        [Header("形状类型")]
        public ProfileShapeType shapeType = ProfileShapeType.Arched;

        public enum ProfileShapeType
        {
            Rectangular,
            Arched,
            Trapezoidal,
            Circular,
            Custom
        }

        [SerializeField]
        private List<Vector3> _customProfilePoints = new List<Vector3>();

        private BezierSpline _profileSpline;
        private BezierSpline _cuttingPathSpline;

        public BezierSpline ProfileSpline => _profileSpline;
        public BezierSpline CuttingPathSpline => _cuttingPathSpline;

        public void GenerateProfile()
        {
            List<Vector3d> profilePoints = new List<Vector3d>();
            Vector3d center = Vector3d.FromVector3(profileCenter);

            switch (shapeType)
            {
                case ProfileShapeType.Rectangular:
                    profilePoints = GenerateRectangularProfile(center);
                    break;
                case ProfileShapeType.Arched:
                    profilePoints = GenerateArchedProfile(center);
                    break;
                case ProfileShapeType.Trapezoidal:
                    profilePoints = GenerateTrapezoidalProfile(center);
                    break;
                case ProfileShapeType.Circular:
                    profilePoints = GenerateCircularProfile(center);
                    break;
                case ProfileShapeType.Custom:
                    profilePoints = GenerateCustomProfile(center);
                    break;
            }

            _profileSpline = new BezierSpline(profilePoints, true);
            _profileSpline.SetLengthSamples(samplesPerSegment * 10);
        }

        public void GenerateCuttingPath(double feedDepth)
        {
            if (_profileSpline == null)
                GenerateProfile();

            feedDepth = Mathd.Clamp(feedDepth, 0, maxFeedDepth);
            int steps = Math.Max(1, (int)Math.Ceiling(feedDepth / feedStep));

            List<Vector3d> pathPoints = new List<Vector3d>();
            int profileSamples = samplesPerSegment * _profileSpline.SegmentCount;

            for (int step = 0; step <= steps; step++)
            {
                double currentFeed = step * feedStep;
                if (currentFeed > feedDepth && step > 0)
                    currentFeed = feedDepth;

                if (step % 2 == 0)
                {
                    for (int i = 0; i <= profileSamples; i++)
                    {
                        double t = (double)i / profileSamples;
                        Vector3d profilePoint = _profileSpline.GetPoint(t);
                        Vector3d pathPoint = profilePoint + new Vector3d(0, 0, currentFeed);
                        pathPoints.Add(pathPoint);
                    }
                }
                else
                {
                    for (int i = profileSamples; i >= 0; i--)
                    {
                        double t = (double)i / profileSamples;
                        Vector3d profilePoint = _profileSpline.GetPoint(t);
                        Vector3d pathPoint = profilePoint + new Vector3d(0, 0, currentFeed);
                        pathPoints.Add(pathPoint);
                    }
                }
            }

            _cuttingPathSpline = new BezierSpline(pathPoints, false);
            _cuttingPathSpline.SetLengthSamples(samplesPerSegment * 5);
        }

        private List<Vector3d> GenerateRectangularProfile(Vector3d center)
        {
            double halfWidth = roadwayWidth * 0.5;
            double halfHeight = roadwayHeight * 0.5;

            List<Vector3d> points = new List<Vector3d>
            {
                center + new Vector3d(-halfWidth, -halfHeight, 0),
                center + new Vector3d(halfWidth, -halfHeight, 0),
                center + new Vector3d(halfWidth, halfHeight, 0),
                center + new Vector3d(-halfWidth, halfHeight, 0)
            };

            return points;
        }

        private List<Vector3d> GenerateArchedProfile(Vector3d center)
        {
            double halfWidth = roadwayWidth * 0.5;
            double sideHeight = roadwayHeight - roofRadius;
            double sideAngleRad = sideAngle * Mathd.Deg2Rad;

            List<Vector3d> points = new List<Vector3d>();

            double bottomY = center.y - roadwayHeight * 0.5 + floorRadius;
            double topY = center.y + roadwayHeight * 0.5 - roofRadius;

            int arcSegments = 12;
            int sideSegments = 4;

            Vector3d bottomLeft = center + new Vector3d(-halfWidth + floorRadius * Mathd.Sin(sideAngleRad), bottomY, 0);
            Vector3d bottomRight = center + new Vector3d(halfWidth - floorRadius * Mathd.Sin(sideAngleRad), bottomY, 0);

            double floorAngleStart = Mathd.PI + sideAngleRad;
            double floorAngleEnd = Mathd.TwoPI - sideAngleRad;
            for (int i = 0; i <= arcSegments; i++)
            {
                double angle = Mathd.Lerp(floorAngleStart, floorAngleEnd, (double)i / arcSegments);
                Vector3d floorPoint = new Vector3d(
                    center.x + Mathd.Cos(angle) * floorRadius,
                    bottomY + Mathd.Sin(angle) * floorRadius,
                    0
                );
                points.Add(floorPoint);
            }

            for (int i = 1; i <= sideSegments; i++)
            {
                double t = (double)i / sideSegments;
                points.Add(Vector3d.Lerp(bottomRight, bottomRight + new Vector3d(0, sideHeight, 0), t));
            }

            double roofAngleStart = -Mathd.HalfPI - sideAngleRad;
            double roofAngleEnd = -Mathd.HalfPI + sideAngleRad;
            Vector3d roofCenter = center + new Vector3d(0, topY, 0);
            for (int i = 1; i <= arcSegments; i++)
            {
                double angle = Mathd.Lerp(roofAngleStart, roofAngleEnd, (double)i / arcSegments);
                Vector3d roofPoint = new Vector3d(
                    roofCenter.x + Mathd.Cos(angle) * roofRadius,
                    roofCenter.y + Mathd.Sin(angle) * roofRadius,
                    0
                );
                points.Add(roofPoint);
            }

            for (int i = 1; i <= sideSegments; i++)
            {
                double t = (double)i / sideSegments;
                points.Add(Vector3d.Lerp(bottomLeft + new Vector3d(0, sideHeight, 0), bottomLeft, t));
            }

            return points;
        }

        private List<Vector3d> GenerateTrapezoidalProfile(Vector3d center)
        {
            double halfBottomWidth = roadwayWidth * 0.5;
            double halfTopWidth = roadwayWidth * 0.4;
            double halfHeight = roadwayHeight * 0.5;
            double sideAngleRad = sideAngle * Mathd.Deg2Rad;

            List<Vector3d> points = new List<Vector3d>
            {
                center + new Vector3d(-halfBottomWidth + floorRadius, -halfHeight, 0),
                center + new Vector3d(halfBottomWidth - floorRadius, -halfHeight, 0),
                center + new Vector3d(halfTopWidth, halfHeight, 0),
                center + new Vector3d(-halfTopWidth, halfHeight, 0)
            };

            return points;
        }

        private List<Vector3d> GenerateCircularProfile(Vector3d center)
        {
            double radius = Mathd.Min(roadwayWidth, roadwayHeight) * 0.5;
            int segments = 32;

            List<Vector3d> points = new List<Vector3d>();
            for (int i = 0; i < segments; i++)
            {
                double angle = (double)i / segments * Mathd.TwoPI;
                points.Add(center + new Vector3d(
                    Mathd.Cos(angle) * radius,
                    Mathd.Sin(angle) * radius,
                    0
                ));
            }

            return points;
        }

        private List<Vector3d> GenerateCustomProfile(Vector3d center)
        {
            List<Vector3d> points = new List<Vector3d>();
            foreach (var point in _customProfilePoints)
            {
                points.Add(Vector3d.FromVector3(point) + center);
            }
            return points.Count >= 2 ? points : GenerateArchedProfile(center);
        }

        public void AddCustomProfilePoint(Vector3 point)
        {
            _customProfilePoints.Add(point);
        }

        public void ClearCustomProfilePoints()
        {
            _customProfilePoints.Clear();
        }

        public List<Vector3d> GetProfilePoints()
        {
            return _profileSpline?.GetPoints(samplesPerSegment) ?? new List<Vector3d>();
        }

        public List<Vector3d> GetCuttingPathPoints()
        {
            return _cuttingPathSpline?.GetPoints(samplesPerSegment) ?? new List<Vector3d>();
        }

        private void OnDrawGizmos()
        {
            if (_profileSpline == null) return;

            Gizmos.color = Color.cyan;
            var profilePoints = _profileSpline.GetPoints(samplesPerSegment);
            for (int i = 0; i < profilePoints.Count; i++)
            {
                int next = (i + 1) % profilePoints.Count;
                Gizmos.DrawLine(profilePoints[i].ToVector3(), profilePoints[next].ToVector3());
            }

            if (_cuttingPathSpline != null)
            {
                Gizmos.color = Color.yellow;
                var pathPoints = _cuttingPathSpline.GetPoints(samplesPerSegment);
                for (int i = 0; i < pathPoints.Count - 1; i++)
                {
                    Gizmos.DrawLine(pathPoints[i].ToVector3(), pathPoints[i + 1].ToVector3());
                }
            }
        }
    }
}
