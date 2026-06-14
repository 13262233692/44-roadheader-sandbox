using System;

namespace RoadheaderSandbox.Core.Math
{
    [Serializable]
    public struct Quaterniond : IEquatable<Quaterniond>
    {
        public double x;
        public double y;
        public double z;
        public double w;

        public Quaterniond(double x, double y, double z, double w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        public static readonly Quaterniond Identity = new Quaterniond(0, 0, 0, 1);

        public double Magnitude => Mathd.Sqrt(x * x + y * y + z * z + w * w);
        public double SqrMagnitude => x * x + y * y + z * z + w * w;

        public Quaterniond Normalized
        {
            get
            {
                double mag = Magnitude;
                if (mag < Mathd.Epsilon) return Identity;
                return new Quaterniond(x / mag, y / mag, z / mag, w / mag);
            }
        }

        public Vector3d EulerAngles
        {
            get
            {
                double sqw = w * w;
                double sqx = x * x;
                double sqy = y * y;
                double sqz = z * z;
                double unit = sqx + sqy + sqz + sqw;
                double test = x * y + z * w;
                Vector3d result = new Vector3d();

                if (test > 0.499 * unit)
                {
                    result.y = 2.0 * Mathd.Atan2(x, w);
                    result.z = Mathd.HalfPI;
                    result.x = 0;
                }
                else if (test < -0.499 * unit)
                {
                    result.y = -2.0 * Mathd.Atan2(x, w);
                    result.z = -Mathd.HalfPI;
                    result.x = 0;
                }
                else
                {
                    result.y = Mathd.Atan2(2.0 * y * w - 2.0 * x * z, sqx - sqy - sqz + sqw);
                    result.z = Mathd.Asin(2.0 * test / unit);
                    result.x = Mathd.Atan2(2.0 * x * w - 2.0 * y * z, -sqx + sqy - sqz + sqw);
                }

                result.x *= Mathd.Rad2Deg;
                result.y *= Mathd.Rad2Deg;
                result.z *= Mathd.Rad2Deg;
                return result;
            }
        }

        public Quaterniond Inverse
        {
            get
            {
                double num = x * x + y * y + z * z + w * w;
                if (num < Mathd.Epsilon) return Identity;
                return new Quaterniond(-x / num, -y / num, -z / num, w / num);
            }
        }

        public static Quaterniond FromToRotation(Vector3d fromDirection, Vector3d toDirection)
        {
            fromDirection = fromDirection.Normalized;
            toDirection = toDirection.Normalized;

            double d = Vector3d.Dot(fromDirection, toDirection);
            if (d >= 1.0 - Mathd.Epsilon) return Identity;
            if (d <= -(1.0 - Mathd.Epsilon))
            {
                Vector3d axis = Vector3d.Cross(Vector3d.Right, fromDirection);
                if (axis.SqrMagnitude < Mathd.Epsilon)
                    axis = Vector3d.Cross(Vector3d.Up, fromDirection);
                return AxisAngle(axis.Normalized, Mathd.PI);
            }

            double s = Mathd.Sqrt((1.0 + d) * 2.0);
            Vector3d c = Vector3d.Cross(fromDirection, toDirection) / s;
            return new Quaterniond(c.x, c.y, c.z, s * 0.5).Normalized;
        }

        public static Quaterniond AxisAngle(Vector3d axis, double angle)
        {
            axis = axis.Normalized;
            double halfAngle = angle * 0.5;
            double sin = Mathd.Sin(halfAngle);
            double cos = Mathd.Cos(halfAngle);
            return new Quaterniond(axis.x * sin, axis.y * sin, axis.z * sin, cos);
        }

        public static Quaterniond Euler(double x, double y, double z)
        {
            return Euler(new Vector3d(x, y, z));
        }

        public static Quaterniond Euler(Vector3d euler)
        {
            euler *= Mathd.Deg2Rad;
            double cx = Mathd.Cos(euler.x * 0.5);
            double sx = Mathd.Sin(euler.x * 0.5);
            double cy = Mathd.Cos(euler.y * 0.5);
            double sy = Mathd.Sin(euler.y * 0.5);
            double cz = Mathd.Cos(euler.z * 0.5);
            double sz = Mathd.Sin(euler.z * 0.5);

            return new Quaterniond(
                sx * cy * cz - cx * sy * sz,
                cx * sy * cz + sx * cy * sz,
                cx * cy * sz - sx * sy * cz,
                cx * cy * cz + sx * sy * sz
            );
        }

        public static Quaterniond LookRotation(Vector3d forward)
        {
            return LookRotation(forward, Vector3d.Up);
        }

        public static Quaterniond LookRotation(Vector3d forward, Vector3d up)
        {
            forward = forward.Normalized;
            Vector3d right = Vector3d.Cross(up, forward).Normalized;
            up = Vector3d.Cross(forward, right).Normalized;

            double m00 = right.x;
            double m01 = right.y;
            double m02 = right.z;
            double m10 = up.x;
            double m11 = up.y;
            double m12 = up.z;
            double m20 = forward.x;
            double m21 = forward.y;
            double m22 = forward.z;

            double tr = m00 + m11 + m22;
            Quaterniond q = new Quaterniond();

            if (tr > 0)
            {
                double s = Mathd.Sqrt(tr + 1.0) * 2.0;
                q.w = 0.25 * s;
                q.x = (m12 - m21) / s;
                q.y = (m20 - m02) / s;
                q.z = (m01 - m10) / s;
            }
            else if (m00 > m11 && m00 > m22)
            {
                double s = Mathd.Sqrt(1.0 + m00 - m11 - m22) * 2.0;
                q.w = (m12 - m21) / s;
                q.x = 0.25 * s;
                q.y = (m10 + m01) / s;
                q.z = (m20 + m02) / s;
            }
            else if (m11 > m22)
            {
                double s = Mathd.Sqrt(1.0 + m11 - m00 - m22) * 2.0;
                q.w = (m20 - m02) / s;
                q.x = (m10 + m01) / s;
                q.y = 0.25 * s;
                q.z = (m21 + m12) / s;
            }
            else
            {
                double s = Mathd.Sqrt(1.0 + m22 - m00 - m11) * 2.0;
                q.w = (m01 - m10) / s;
                q.x = (m20 + m02) / s;
                q.y = (m21 + m12) / s;
                q.z = 0.25 * s;
            }

            return q.Normalized;
        }

        public static Quaterniond Lerp(Quaterniond a, Quaterniond b, double t)
        {
            t = Mathd.Clamp01(t);
            return new Quaterniond(
                a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t,
                a.z + (b.z - a.z) * t,
                a.w + (b.w - a.w) * t
            ).Normalized;
        }

        public static Quaterniond Slerp(Quaterniond a, Quaterniond b, double t)
        {
            t = Mathd.Clamp01(t);
            a = a.Normalized;
            b = b.Normalized;

            double dot = Dot(a, b);

            if (dot < 0.0)
            {
                b = new Quaterniond(-b.x, -b.y, -b.z, -b.w);
                dot = -dot;
            }

            if (dot > 0.9995)
            {
                return Lerp(a, b, t);
            }

            double theta = Mathd.Acos(dot);
            double sinTheta = Mathd.Sin(theta);
            double ratioA = Mathd.Sin((1.0 - t) * theta) / sinTheta;
            double ratioB = Mathd.Sin(t * theta) / sinTheta;

            return new Quaterniond(
                a.x * ratioA + b.x * ratioB,
                a.y * ratioA + b.y * ratioB,
                a.z * ratioA + b.z * ratioB,
                a.w * ratioA + b.w * ratioB
            );
        }

        public static double Dot(Quaterniond a, Quaterniond b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
        }

        public static double Angle(Quaterniond a, Quaterniond b)
        {
            double dot = Mathd.Abs(Dot(a, b));
            return Mathd.Acos(Mathd.Clamp(dot, -1.0, 1.0)) * 2.0 * Mathd.Rad2Deg;
        }

        public void Normalize()
        {
            double mag = Magnitude;
            if (mag < Mathd.Epsilon)
            {
                this = Identity;
                return;
            }
            x /= mag;
            y /= mag;
            z /= mag;
            w /= mag;
        }

        public static Quaterniond operator *(Quaterniond a, Quaterniond b)
        {
            return new Quaterniond(
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z,
                a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
            );
        }

        public static Vector3d operator *(Quaterniond rotation, Vector3d point)
        {
            double num1 = rotation.x * 2.0;
            double num2 = rotation.y * 2.0;
            double num3 = rotation.z * 2.0;
            double num4 = rotation.x * num1;
            double num5 = rotation.y * num2;
            double num6 = rotation.z * num3;
            double num7 = rotation.x * num2;
            double num8 = rotation.x * num3;
            double num9 = rotation.y * num3;
            double num10 = rotation.w * num1;
            double num11 = rotation.w * num2;
            double num12 = rotation.w * num3;

            Vector3d result = new Vector3d();
            result.x = (1.0 - (num5 + num6)) * point.x + (num7 - num12) * point.y + (num8 + num11) * point.z;
            result.y = (num7 + num12) * point.x + (1.0 - (num4 + num6)) * point.y + (num9 - num10) * point.z;
            result.z = (num8 - num11) * point.x + (num9 + num10) * point.y + (1.0 - (num4 + num5)) * point.z;
            return result;
        }

        public static bool operator ==(Quaterniond left, Quaterniond right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Quaterniond left, Quaterniond right)
        {
            return !left.Equals(right);
        }

        public bool Equals(Quaterniond other)
        {
            const double epsilon = 1e-12;
            return Mathd.Abs(x - other.x) < epsilon &&
                   Mathd.Abs(y - other.y) < epsilon &&
                   Mathd.Abs(z - other.z) < epsilon &&
                   Mathd.Abs(w - other.w) < epsilon;
        }

        public override bool Equals(object obj)
        {
            return obj is Quaterniond other && Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hashCode = x.GetHashCode();
                hashCode = (hashCode * 397) ^ y.GetHashCode();
                hashCode = (hashCode * 397) ^ z.GetHashCode();
                hashCode = (hashCode * 397) ^ w.GetHashCode();
                return hashCode;
            }
        }

        public override string ToString()
        {
            return $"({x:F6}, {y:F6}, {z:F6}, {w:F6})";
        }

        public static Quaterniond FromQuaternion(UnityEngine.Quaternion q)
        {
            return new Quaterniond(q.x, q.y, q.z, q.w);
        }

        public UnityEngine.Quaternion ToQuaternion()
        {
            return new UnityEngine.Quaternion((float)x, (float)y, (float)z, (float)w);
        }
    }
}
