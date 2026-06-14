using System;
using System.Collections.Generic;
using RoadheaderSandbox.Core.Math;

namespace RoadheaderSandbox.Core.Math
{
    [Serializable]
    public class BezierSpline
    {
        private readonly List<BezierCurve> _segments = new List<BezierCurve>();
        private readonly List<double> _segmentLengths = new List<double>();
        private double _totalLength;
        private int _lengthSamples = 100;

        public IReadOnlyList<BezierCurve> Segments => _segments;
        public int SegmentCount => _segments.Count;
        public double TotalLength => _totalLength;
        public bool IsLoop { get; set; }

        public BezierSpline()
        {
        }

        public BezierSpline(IEnumerable<Vector3d> points, bool loop = false)
        {
            IsLoop = loop;
            CreateFromPoints(points);
        }

        public void CreateFromPoints(IEnumerable<Vector3d> points)
        {
            _segments.Clear();
            _segmentLengths.Clear();

            List<Vector3d> pointList = new List<Vector3d>(points);
            if (pointList.Count < 2)
                throw new ArgumentException("At least 2 points are required to create a spline.");

            if (IsLoop && pointList.Count >= 2)
            {
                Vector3d first = pointList[0];
                Vector3d last = pointList[pointList.Count - 1];
                if (first != last)
                    pointList.Add(first);
            }

            for (int i = 0; i < pointList.Count - 1; i++)
            {
                Vector3d p0 = pointList[i];
                Vector3d p3 = pointList[i + 1];

                Vector3d prevPoint = i > 0 ? pointList[i - 1] : p0;
                Vector3d nextPoint = i < pointList.Count - 2 ? pointList[i + 2] : p3;

                Vector3d tangentStart = (p3 - prevPoint).Normalized;
                Vector3d tangentEnd = (nextPoint - p0).Normalized;

                double segmentLength = Vector3d.Distance(p0, p3);
                double handleLength = segmentLength * 0.333;

                Vector3d p1 = p0 + tangentStart * handleLength;
                Vector3d p2 = p3 - tangentEnd * handleLength;

                _segments.Add(new BezierCurve(p0, p1, p2, p3));
            }

            CalculateLengths();
        }

        public void AddSegment(BezierCurve curve)
        {
            _segments.Add(curve);
            CalculateLengths();
        }

        public void AddSegment(Vector3d p0, Vector3d p1, Vector3d p2, Vector3d p3)
        {
            _segments.Add(new BezierCurve(p0, p1, p2, p3));
            CalculateLengths();
        }

        public void RemoveSegment(int index)
        {
            if (index >= 0 && index < _segments.Count)
            {
                _segments.RemoveAt(index);
                CalculateLengths();
            }
        }

        public void Clear()
        {
            _segments.Clear();
            _segmentLengths.Clear();
            _totalLength = 0;
        }

        private void CalculateLengths()
        {
            _segmentLengths.Clear();
            _totalLength = 0;

            foreach (var segment in _segments)
            {
                double length = segment.EstimateLength(_lengthSamples);
                _segmentLengths.Add(length);
                _totalLength += length;
            }
        }

        public void SetLengthSamples(int samples)
        {
            _lengthSamples = Math.Max(10, samples);
            CalculateLengths();
        }

        private (int segmentIndex, double localT) GetSegmentIndexAndT(double t)
        {
            if (_segments.Count == 0)
                return (0, 0);

            if (IsLoop)
                t = Mathd.Repeat(t, 1.0);
            else
                t = Mathd.Clamp01(t);

            if (_segments.Count == 1)
                return (0, t);

            double targetLength = t * _totalLength;
            double accumulated = 0;

            for (int i = 0; i < _segments.Count; i++)
            {
                if (accumulated + _segmentLengths[i] >= targetLength)
                {
                    double localLength = targetLength - accumulated;
                    double localT = _segments[i].GetParameterAtLength(localLength, _lengthSamples);
                    return (i, localT);
                }
                accumulated += _segmentLengths[i];
            }

            return (_segments.Count - 1, 1.0);
        }

        public Vector3d GetPoint(double t)
        {
            var (segmentIndex, localT) = GetSegmentIndexAndT(t);
            return _segments[segmentIndex].GetPoint(localT);
        }

        public Vector3d GetTangent(double t)
        {
            var (segmentIndex, localT) = GetSegmentIndexAndT(t);
            return _segments[segmentIndex].GetTangent(localT);
        }

        public Vector3d GetNormal(double t, Vector3d up)
        {
            var (segmentIndex, localT) = GetSegmentIndexAndT(t);
            return _segments[segmentIndex].GetNormal(localT, up);
        }

        public Quaterniond GetOrientation(double t, Vector3d up)
        {
            var (segmentIndex, localT) = GetSegmentIndexAndT(t);
            return _segments[segmentIndex].GetOrientation(localT, up);
        }

        public Vector3d GetPointAtDistance(double distance)
        {
            double t = distance / _totalLength;
            return GetPoint(t);
        }

        public Vector3d GetTangentAtDistance(double distance)
        {
            double t = distance / _totalLength;
            return GetTangent(t);
        }

        public List<Vector3d> GetPoints(int samplesPerSegment)
        {
            List<Vector3d> points = new List<Vector3d>();
            foreach (var segment in _segments)
            {
                var segmentPoints = segment.GetPoints(samplesPerSegment);
                if (points.Count > 0)
                    segmentPoints.RemoveAt(0);
                points.AddRange(segmentPoints);
            }
            return points;
        }

        public double GetParameterAtDistance(double distance)
        {
            return Mathd.Clamp01(distance / _totalLength);
        }

        public void Transform(Matrix4x4d matrix)
        {
            foreach (var segment in _segments)
            {
                segment.Transform(matrix);
            }
            CalculateLengths();
        }

        public BezierSpline Clone()
        {
            BezierSpline clone = new BezierSpline
            {
                IsLoop = this.IsLoop,
                _lengthSamples = this._lengthSamples
            };

            foreach (var segment in _segments)
            {
                clone._segments.Add(segment.Clone());
            }

            clone.CalculateLengths();
            return clone;
        }
    }
}
