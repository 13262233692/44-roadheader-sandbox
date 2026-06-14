using System;
using System.Runtime.InteropServices;

namespace RoadheaderSandbox.Core.Math
{
    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct Vector3d : IEquatable<Vector3d>
    {
        public double x;
        public double y;
        public double z;

        public Vector3d(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public static readonly Vector3d Zero = new Vector3d(0, 0, 0);
        public static readonly Vector3d One = new Vector3d(1, 1, 1);
        public static readonly Vector3d Forward = new Vector3d(0, 0, 1);
        public static readonly Vector3d Backward = new Vector3d(0, 0, -1);
        public static readonly Vector3d Up = new Vector3d(0, 1, 0);
        public static readonly Vector3d Down = new Vector3d(0, -1, 0);
        public static readonly Vector3d Right = new Vector3d(1, 0, 0);
        public static readonly Vector3d Left = new Vector3d(-1, 0, 0);

        public double Magnitude => System.Math.Sqrt(x * x + y * y + z * z);
        public double SqrMagnitude => x * x + y * y + z * z;

        public Vector3d Normalized
        {
            get
            {
                double mag = Magnitude;
                if (mag < 1e-15) return Zero;
                return new Vector3d(x / mag, y / mag, z / mag);
            }
        }

        public static double Dot(Vector3d a, Vector3d b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        public static Vector3d Cross(Vector3d a, Vector3d b)
        {
            return new Vector3d(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            );
        }

        public static double Distance(Vector3d a, Vector3d b)
        {
            return (a - b).Magnitude;
        }

        public static Vector3d Lerp(Vector3d a, Vector3d b, double t)
        {
            t = Mathd.Clamp01(t);
            return new Vector3d(
                a.x + (b.x - a.x) * t,
                a.y + (b.y - a.y) * t,
                a.z + (b.z - a.z) * t
            );
        }

        public static Vector3d operator +(Vector3d a, Vector3d b)
        {
            return new Vector3d(a.x + b.x, a.y + b.y, a.z + b.z);
        }

        public static Vector3d operator -(Vector3d a, Vector3d b)
        {
            return new Vector3d(a.x - b.x, a.y - b.y, a.z - b.z);
        }

        public static Vector3d operator -(Vector3d v)
        {
            return new Vector3d(-v.x, -v.y, -v.z);
        }

        public static Vector3d operator *(Vector3d v, double scalar)
        {
            return new Vector3d(v.x * scalar, v.y * scalar, v.z * scalar);
        }

        public static Vector3d operator *(double scalar, Vector3d v)
        {
            return new Vector3d(v.x * scalar, v.y * scalar, v.z * scalar);
        }

        public static Vector3d operator /(Vector3d v, double scalar)
        {
            return new Vector3d(v.x / scalar, v.y / scalar, v.z / scalar);
        }

        public static bool operator ==(Vector3d left, Vector3d right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Vector3d left, Vector3d right)
        {
            return !left.Equals(right);
        }

        public bool Equals(Vector3d other)
        {
            const double epsilon = 1e-12;
            return System.Math.Abs(x - other.x) < epsilon &&
                   System.Math.Abs(y - other.y) < epsilon &&
                   System.Math.Abs(z - other.z) < epsilon;
        }

        public override bool Equals(object obj)
        {
            return obj is Vector3d other && Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hashCode = x.GetHashCode();
                hashCode = (hashCode * 397) ^ y.GetHashCode();
                hashCode = (hashCode * 397) ^ z.GetHashCode();
                return hashCode;
            }
        }

        public override string ToString()
        {
            return $"({x:F6}, {y:F6}, {z:F6})";
        }

        public UnityEngine.Vector3 ToVector3()
        {
            return new UnityEngine.Vector3((float)x, (float)y, (float)z);
        }

        public static Vector3d FromVector3(UnityEngine.Vector3 v)
        {
            return new Vector3d(v.x, v.y, v.z);
        }
    }
}
