using System;
using System.Collections.Generic;
using RoadheaderSandbox.Core.Math;

namespace RoadheaderSandbox.Core.Math
{
    [Serializable]
    public class BezierCurve
    {
        public Vector3d P0;
        public Vector3d P1;
        public Vector3d P2;
        public Vector3d P3;

        public BezierCurve(Vector3d p0, Vector3d p1, Vector3d p2, Vector3d p3)
        {
            P0 = p0;
            P1 = p1;
            P2 = p2;
            P3 = p3;
        }

        public Vector3d GetPoint(double t)
        {
            t = Mathd.Clamp01(t);
            double oneMinusT = 1.0 - t;
            double oneMinusT2 = oneMinusT * oneMinusT;
            double oneMinusT3 = oneMinusT2 * oneMinusT;
            double t2 = t * t;
            double t3 = t2 * t;

            return oneMinusT3 * P0 +
                   3.0 * oneMinusT2 * t * P1 +
                   3.0 * oneMinusT * t2 * P2 +
                   t3 * P3;
        }

        public Vector3d GetTangent(double t)
        {
            t = Mathd.Clamp01(t);
            double oneMinusT = 1.0 - t;
            double oneMinusT2 = oneMinusT * oneMinusT;
            double t2 = t * t;

            Vector3d tangent =
                3.0 * oneMinusT2 * (P1 - P0) +
                6.0 * oneMinusT * t * (P2 - P1) +
                3.0 * t2 * (P3 - P2);

            return tangent.Normalized;
        }

        public Vector3d GetSecondDerivative(double t)
        {
            t = Mathd.Clamp01(t);
            double oneMinusT = 1.0 - t;

            return 6.0 * oneMinusT * (P2 - 2.0 * P1 + P0) +
                   6.0 * t * (P3 - 2.0 * P2 + P1);
        }

        public Vector3d GetNormal(double t, Vector3d up)
        {
            Vector3d tangent = GetTangent(t);
            Vector3d secondDeriv = GetSecondDerivative(t);
            Vector3d binormal = Vector3d.Cross(tangent, secondDeriv);
            if (binormal.SqrMagnitude < Mathd.Epsilon)
            {
                binormal = Vector3d.Cross(tangent, up).Normalized;
            }
            else
            {
                binormal = binormal.Normalized;
            }
            return Vector3d.Cross(binormal, tangent).Normalized;
        }

        public Quaterniond GetOrientation(double t, Vector3d up)
        {
            Vector3d tangent = GetTangent(t);
            Vector3d normal = GetNormal(t, up);
            Vector3d binormal = Vector3d.Cross(tangent, normal).Normalized;

            Matrix4x4d rotMatrix = new Matrix4x4d();
            rotMatrix.SetColumn(0, normal);
            rotMatrix.SetColumn(1, binormal);
            rotMatrix.SetColumn(2, tangent);
            rotMatrix.m33 = 1.0;

            return QuaternionFromMatrix(rotMatrix);
        }

        private Quaterniond QuaternionFromMatrix(Matrix4x4d m)
        {
            double tr = m.m00 + m.m11 + m.m22;
            Quaterniond q = new Quaterniond();

            if (tr > 0)
            {
                double s = Mathd.Sqrt(tr + 1.0) * 2.0;
                q.w = 0.25 * s;
                q.x = (m.m12 - m.m21) / s;
                q.y = (m.m20 - m.m02) / s;
                q.z = (m.m01 - m.m10) / s;
            }
            else if (m.m00 > m.m11 && m.m00 > m.m22)
            {
                double s = Mathd.Sqrt(1.0 + m.m00 - m.m11 - m.m22) * 2.0;
                q.w = (m.m12 - m.m21) / s;
                q.x = 0.25 * s;
                q.y = (m.m10 + m.m01) / s;
                q.z = (m.m20 + m.m02) / s;
            }
            else if (m.m11 > m.m22)
            {
                double s = Mathd.Sqrt(1.0 + m.m11 - m.m00 - m.m22) * 2.0;
                q.w = (m.m20 - m.m02) / s;
                q.x = (m.m10 + m.m01) / s;
                q.y = 0.25 * s;
                q.z = (m.m21 + m.m12) / s;
            }
            else
            {
                double s = Mathd.Sqrt(1.0 + m.m22 - m.m00 - m.m11) * 2.0;
                q.w = (m.m01 - m.m10) / s;
                q.x = (m.m20 + m.m02) / s;
                q.y = (m.m21 + m.m12) / s;
                q.z = 0.25 * s;
            }

            return q.Normalized;
        }

        public double EstimateLength(int samples = 100)
        {
            double length = 0;
            Vector3d prev = GetPoint(0);
            for (int i = 1; i <= samples; i++)
            {
                double t = (double)i / samples;
                Vector3d curr = GetPoint(t);
                length += Vector3d.Distance(prev, curr);
                prev = curr;
            }
            return length;
        }

        public double GetParameterAtLength(double targetLength, int samples = 100)
        {
            double totalLength = EstimateLength(samples);
            targetLength = Mathd.Clamp(targetLength, 0, totalLength);

            double accumulatedLength = 0;
            Vector3d prev = GetPoint(0);

            for (int i = 1; i <= samples; i++)
            {
                double t = (double)i / samples;
                Vector3d curr = GetPoint(t);
                double segmentLength = Vector3d.Distance(prev, curr);

                if (accumulatedLength + segmentLength >= targetLength)
                {
                    double remaining = targetLength - accumulatedLength;
                    double tPrev = (double)(i - 1) / samples;
                    double tNext = t;
                    return Mathd.Lerp(tPrev, tNext, remaining / segmentLength);
                }

                accumulatedLength += segmentLength;
                prev = curr;
            }

            return 1.0;
        }

        public List<Vector3d> GetPoints(int count)
        {
            List<Vector3d> points = new List<Vector3d>();
            for (int i = 0; i <= count; i++)
            {
                points.Add(GetPoint((double)i / count));
            }
            return points;
        }

        public BezierCurve Clone()
        {
            return new BezierCurve(P0, P1, P2, P3);
        }

        public void Transform(Matrix4x4d matrix)
        {
            P0 = matrix.MultiplyPoint(P0);
            P1 = matrix.MultiplyPoint(P1);
            P2 = matrix.MultiplyPoint(P2);
            P3 = matrix.MultiplyPoint(P3);
        }
    }
}
