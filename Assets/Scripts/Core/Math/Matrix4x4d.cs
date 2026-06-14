using System;

namespace RoadheaderSandbox.Core.Math
{
    [Serializable]
    public struct Matrix4x4d : IEquatable<Matrix4x4d>
    {
        public double m00, m01, m02, m03;
        public double m10, m11, m12, m13;
        public double m20, m21, m22, m23;
        public double m30, m31, m32, m33;

        public static readonly Matrix4x4d Identity = new Matrix4x4d
        {
            m00 = 1, m01 = 0, m02 = 0, m03 = 0,
            m10 = 0, m11 = 1, m12 = 0, m13 = 0,
            m20 = 0, m21 = 0, m22 = 1, m23 = 0,
            m30 = 0, m31 = 0, m32 = 0, m33 = 1
        };

        public static readonly Matrix4x4d Zero = new Matrix4x4d();

        public double this[int row, int column]
        {
            get
            {
                switch (row * 4 + column)
                {
                    case 0: return m00; case 1: return m01; case 2: return m02; case 3: return m03;
                    case 4: return m10; case 5: return m11; case 6: return m12; case 7: return m13;
                    case 8: return m20; case 9: return m21; case 10: return m22; case 11: return m23;
                    case 12: return m30; case 13: return m31; case 14: return m32; case 15: return m33;
                    default: throw new IndexOutOfRangeException();
                }
            }
            set
            {
                switch (row * 4 + column)
                {
                    case 0: m00 = value; break; case 1: m01 = value; break; case 2: m02 = value; break; case 3: m03 = value; break;
                    case 4: m10 = value; break; case 5: m11 = value; break; case 6: m12 = value; break; case 7: m13 = value; break;
                    case 8: m20 = value; break; case 9: m21 = value; break; case 10: m22 = value; break; case 11: m23 = value; break;
                    case 12: m30 = value; break; case 13: m31 = value; break; case 14: m32 = value; break; case 15: m33 = value; break;
                    default: throw new IndexOutOfRangeException();
                }
            }
        }

        public double Determinant
        {
            get
            {
                double det2_01_01 = m00 * m11 - m01 * m10;
                double det2_01_02 = m00 * m12 - m02 * m10;
                double det2_01_03 = m00 * m13 - m03 * m10;
                double det2_01_12 = m01 * m12 - m02 * m11;
                double det2_01_13 = m01 * m13 - m03 * m11;
                double det2_01_23 = m02 * m13 - m03 * m12;
                double det3_201_012 = m20 * det2_01_12 - m21 * det2_01_02 + m22 * det2_01_01;
                double det3_201_013 = m20 * det2_01_13 - m21 * det2_01_03 + m23 * det2_01_01;
                double det3_201_023 = m20 * det2_01_23 - m22 * det2_01_03 + m23 * det2_01_02;
                double det3_201_123 = m21 * det2_01_23 - m22 * det2_01_13 + m23 * det2_01_12;
                return -(m30 * det3_201_123 - m31 * det3_201_023 + m32 * det3_201_013 - m33 * det3_201_012);
            }
        }

        public Matrix4x4d Inverse
        {
            get
            {
                Matrix4x4d result = this;
                if (result.Invert()) return result;
                return Identity;
            }
        }

        public Matrix4x4d Transpose
        {
            get
            {
                return new Matrix4x4d
                {
                    m00 = m00, m01 = m10, m02 = m20, m03 = m30,
                    m10 = m01, m11 = m11, m12 = m21, m13 = m31,
                    m20 = m02, m21 = m12, m22 = m22, m23 = m32,
                    m30 = m03, m31 = m13, m32 = m23, m33 = m33
                };
            }
        }

        public Vector3d GetColumn(int index)
        {
            switch (index)
            {
                case 0: return new Vector3d(m00, m10, m20);
                case 1: return new Vector3d(m01, m11, m21);
                case 2: return new Vector3d(m02, m12, m22);
                case 3: return new Vector3d(m03, m13, m23);
                default: throw new IndexOutOfRangeException();
            }
        }

        public Vector3d GetRow(int index)
        {
            switch (index)
            {
                case 0: return new Vector3d(m00, m01, m02);
                case 1: return new Vector3d(m10, m11, m12);
                case 2: return new Vector3d(m20, m21, m22);
                case 3: return new Vector3d(m30, m31, m32);
                default: throw new IndexOutOfRangeException();
            }
        }

        public void SetColumn(int index, Vector3d v)
        {
            switch (index)
            {
                case 0: m00 = v.x; m10 = v.y; m20 = v.z; break;
                case 1: m01 = v.x; m11 = v.y; m21 = v.z; break;
                case 2: m02 = v.x; m12 = v.y; m22 = v.z; break;
                case 3: m03 = v.x; m13 = v.y; m23 = v.z; break;
                default: throw new IndexOutOfRangeException();
            }
        }

        public void SetRow(int index, Vector3d v)
        {
            switch (index)
            {
                case 0: m00 = v.x; m01 = v.y; m02 = v.z; break;
                case 1: m10 = v.x; m11 = v.y; m12 = v.z; break;
                case 2: m20 = v.x; m21 = v.y; m22 = v.z; break;
                case 3: m30 = v.x; m31 = v.y; m32 = v.z; break;
                default: throw new IndexOutOfRangeException();
            }
        }

        public static Matrix4x4d TRS(Vector3d pos, Quaterniond q, Vector3d s)
        {
            Matrix4x4d m = new Matrix4x4d();
            double num = q.x * 2.0;
            double num2 = q.y * 2.0;
            double num3 = q.z * 2.0;
            double num4 = q.x * num;
            double num5 = q.y * num2;
            double num6 = q.z * num3;
            double num7 = q.x * num2;
            double num8 = q.x * num3;
            double num9 = q.y * num3;
            double num10 = q.w * num;
            double num11 = q.w * num2;
            double num12 = q.w * num3;
            m.m00 = 1.0 - (num5 + num6);
            m.m10 = num7 + num12;
            m.m20 = num8 - num11;
            m.m30 = 0.0;
            m.m01 = num7 - num12;
            m.m11 = 1.0 - (num4 + num6);
            m.m21 = num9 + num10;
            m.m31 = 0.0;
            m.m02 = num8 + num11;
            m.m12 = num9 - num10;
            m.m22 = 1.0 - (num4 + num5);
            m.m32 = 0.0;
            m.m03 = pos.x;
            m.m13 = pos.y;
            m.m23 = pos.z;
            m.m33 = 1.0;
            m.m00 *= s.x;
            m.m10 *= s.x;
            m.m20 *= s.x;
            m.m01 *= s.y;
            m.m11 *= s.y;
            m.m21 *= s.y;
            m.m02 *= s.z;
            m.m12 *= s.z;
            m.m22 *= s.z;
            return m;
        }

        public static Matrix4x4d Translate(Vector3d vector)
        {
            Matrix4x4d result = Identity;
            result.m03 = vector.x;
            result.m13 = vector.y;
            result.m23 = vector.z;
            return result;
        }

        public static Matrix4x4d Scale(Vector3d vector)
        {
            Matrix4x4d result = Identity;
            result.m00 = vector.x;
            result.m11 = vector.y;
            result.m22 = vector.z;
            return result;
        }

        public static Matrix4x4d Rotate(Quaterniond q)
        {
            return TRS(Vector3d.Zero, q, Vector3d.One);
        }

        public Vector3d MultiplyPoint(Vector3d point)
        {
            return new Vector3d(
                m00 * point.x + m01 * point.y + m02 * point.z + m03,
                m10 * point.x + m11 * point.y + m12 * point.z + m13,
                m20 * point.x + m21 * point.y + m22 * point.z + m23
            );
        }

        public Vector3d MultiplyVector(Vector3d vector)
        {
            return new Vector3d(
                m00 * vector.x + m01 * vector.y + m02 * vector.z,
                m10 * vector.x + m11 * vector.y + m12 * vector.z,
                m20 * vector.x + m21 * vector.y + m22 * vector.z
            );
        }

        public bool Invert()
        {
            double[] inv = new double[16];
            inv[0] = m11 * m22 * m33 - m11 * m23 * m32 - m21 * m12 * m33 +
                      m21 * m13 * m32 + m31 * m12 * m23 - m31 * m13 * m22;
            inv[4] = -m10 * m22 * m33 + m10 * m23 * m32 + m20 * m12 * m33 -
                      m20 * m13 * m32 - m30 * m12 * m23 + m30 * m13 * m22;
            inv[8] = m10 * m21 * m33 - m10 * m23 * m31 - m20 * m11 * m33 +
                      m20 * m13 * m31 + m30 * m11 * m23 - m30 * m13 * m21;
            inv[12] = -m10 * m21 * m32 + m10 * m23 * m31 + m20 * m11 * m32 -
                      m20 * m13 * m31 - m30 * m11 * m23 + m30 * m13 * m21;
            inv[1] = -m01 * m22 * m33 + m01 * m23 * m32 + m21 * m02 * m33 -
                      m21 * m03 * m32 - m31 * m02 * m23 + m31 * m03 * m22;
            inv[5] = m00 * m22 * m33 - m00 * m23 * m32 - m20 * m02 * m33 +
                      m20 * m03 * m32 + m30 * m02 * m23 - m30 * m03 * m22;
            inv[9] = -m00 * m21 * m33 + m00 * m23 * m31 + m20 * m01 * m33 -
                      m20 * m03 * m31 - m30 * m01 * m23 + m30 * m03 * m21;
            inv[13] = m00 * m21 * m32 - m00 * m23 * m31 - m20 * m01 * m32 +
                      m20 * m03 * m31 + m30 * m01 * m23 - m30 * m03 * m21;
            inv[2] = m01 * m12 * m33 - m01 * m13 * m32 - m11 * m02 * m33 +
                      m11 * m03 * m32 + m31 * m02 * m13 - m31 * m03 * m12;
            inv[6] = -m00 * m12 * m33 + m00 * m13 * m32 + m10 * m02 * m33 -
                      m10 * m03 * m32 - m30 * m02 * m13 + m30 * m03 * m12;
            inv[10] = m00 * m11 * m33 - m00 * m13 * m31 - m10 * m01 * m33 +
                      m10 * m03 * m31 + m30 * m01 * m13 - m30 * m03 * m11;
            inv[14] = -m00 * m11 * m32 + m00 * m13 * m31 + m10 * m01 * m32 -
                      m10 * m03 * m31 - m30 * m01 * m13 + m30 * m03 * m11;
            inv[3] = -m01 * m12 * m23 + m01 * m13 * m22 + m11 * m02 * m23 -
                      m11 * m03 * m22 - m21 * m02 * m13 + m21 * m03 * m12;
            inv[7] = m00 * m12 * m23 - m00 * m13 * m22 - m10 * m02 * m23 +
                      m10 * m03 * m22 + m20 * m02 * m13 - m20 * m03 * m12;
            inv[11] = -m00 * m11 * m23 + m00 * m13 * m21 + m10 * m01 * m23 -
                      m10 * m03 * m21 - m20 * m01 * m13 + m20 * m03 * m11;
            inv[15] = m00 * m11 * m22 - m00 * m13 * m21 - m10 * m01 * m22 +
                      m10 * m03 * m21 + m20 * m01 * m12 - m20 * m03 * m11;

            double det = m00 * inv[0] + m01 * inv[4] + m02 * inv[8] + m03 * inv[12];
            if (Mathd.Abs(det) < Mathd.Epsilon) return false;

            det = 1.0 / det;
            m00 = inv[0] * det; m01 = inv[1] * det; m02 = inv[2] * det; m03 = inv[3] * det;
            m10 = inv[4] * det; m11 = inv[5] * det; m12 = inv[6] * det; m13 = inv[7] * det;
            m20 = inv[8] * det; m21 = inv[9] * det; m22 = inv[10] * det; m23 = inv[11] * det;
            m30 = inv[12] * det; m31 = inv[13] * det; m32 = inv[14] * det; m33 = inv[15] * det;
            return true;
        }

        public static Matrix4x4d operator *(Matrix4x4d a, Matrix4x4d b)
        {
            Matrix4x4d result = new Matrix4x4d();
            result.m00 = a.m00 * b.m00 + a.m01 * b.m10 + a.m02 * b.m20 + a.m03 * b.m30;
            result.m01 = a.m00 * b.m01 + a.m01 * b.m11 + a.m02 * b.m21 + a.m03 * b.m31;
            result.m02 = a.m00 * b.m02 + a.m01 * b.m12 + a.m02 * b.m22 + a.m03 * b.m32;
            result.m03 = a.m00 * b.m03 + a.m01 * b.m13 + a.m02 * b.m23 + a.m03 * b.m33;
            result.m10 = a.m10 * b.m00 + a.m11 * b.m10 + a.m12 * b.m20 + a.m13 * b.m30;
            result.m11 = a.m10 * b.m01 + a.m11 * b.m11 + a.m12 * b.m21 + a.m13 * b.m31;
            result.m12 = a.m10 * b.m02 + a.m11 * b.m12 + a.m12 * b.m22 + a.m13 * b.m32;
            result.m13 = a.m10 * b.m03 + a.m11 * b.m13 + a.m12 * b.m23 + a.m13 * b.m33;
            result.m20 = a.m20 * b.m00 + a.m21 * b.m10 + a.m22 * b.m20 + a.m23 * b.m30;
            result.m21 = a.m20 * b.m01 + a.m21 * b.m11 + a.m22 * b.m21 + a.m23 * b.m31;
            result.m22 = a.m20 * b.m02 + a.m21 * b.m12 + a.m22 * b.m22 + a.m23 * b.m32;
            result.m23 = a.m20 * b.m03 + a.m21 * b.m13 + a.m22 * b.m23 + a.m23 * b.m33;
            result.m30 = a.m30 * b.m00 + a.m31 * b.m10 + a.m32 * b.m20 + a.m33 * b.m30;
            result.m31 = a.m30 * b.m01 + a.m31 * b.m11 + a.m32 * b.m21 + a.m33 * b.m31;
            result.m32 = a.m30 * b.m02 + a.m31 * b.m12 + a.m32 * b.m22 + a.m33 * b.m32;
            result.m33 = a.m30 * b.m03 + a.m31 * b.m13 + a.m32 * b.m23 + a.m33 * b.m33;
            return result;
        }

        public static Vector3d operator *(Matrix4x4d m, Vector3d v)
        {
            return m.MultiplyPoint(v);
        }

        public bool Equals(Matrix4x4d other)
        {
            const double epsilon = 1e-12;
            return
                Mathd.Abs(m00 - other.m00) < epsilon && Mathd.Abs(m01 - other.m01) < epsilon &&
                Mathd.Abs(m02 - other.m02) < epsilon && Mathd.Abs(m03 - other.m03) < epsilon &&
                Mathd.Abs(m10 - other.m10) < epsilon && Mathd.Abs(m11 - other.m11) < epsilon &&
                Mathd.Abs(m12 - other.m12) < epsilon && Mathd.Abs(m13 - other.m13) < epsilon &&
                Mathd.Abs(m20 - other.m20) < epsilon && Mathd.Abs(m21 - other.m21) < epsilon &&
                Mathd.Abs(m22 - other.m22) < epsilon && Mathd.Abs(m23 - other.m23) < epsilon &&
                Mathd.Abs(m30 - other.m30) < epsilon && Mathd.Abs(m31 - other.m31) < epsilon &&
                Mathd.Abs(m32 - other.m32) < epsilon && Mathd.Abs(m33 - other.m33) < epsilon;
        }

        public override bool Equals(object obj)
        {
            return obj is Matrix4x4d other && Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hashCode = m00.GetHashCode();
                hashCode = (hashCode * 397) ^ m01.GetHashCode();
                hashCode = (hashCode * 397) ^ m02.GetHashCode();
                hashCode = (hashCode * 397) ^ m03.GetHashCode();
                hashCode = (hashCode * 397) ^ m10.GetHashCode();
                hashCode = (hashCode * 397) ^ m11.GetHashCode();
                hashCode = (hashCode * 397) ^ m12.GetHashCode();
                hashCode = (hashCode * 397) ^ m13.GetHashCode();
                hashCode = (hashCode * 397) ^ m20.GetHashCode();
                hashCode = (hashCode * 397) ^ m21.GetHashCode();
                hashCode = (hashCode * 397) ^ m22.GetHashCode();
                hashCode = (hashCode * 397) ^ m23.GetHashCode();
                hashCode = (hashCode * 397) ^ m30.GetHashCode();
                hashCode = (hashCode * 397) ^ m31.GetHashCode();
                hashCode = (hashCode * 397) ^ m32.GetHashCode();
                hashCode = (hashCode * 397) ^ m33.GetHashCode();
                return hashCode;
            }
        }

        public static bool operator ==(Matrix4x4d left, Matrix4x4d right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Matrix4x4d left, Matrix4x4d right)
        {
            return !left.Equals(right);
        }

        public override string ToString()
        {
            return string.Format(
                "[{0:F6}, {1:F6}, {2:F6}, {3:F6}]\n" +
                "[{4:F6}, {5:F6}, {6:F6}, {7:F6}]\n" +
                "[{8:F6}, {9:F6}, {10:F6}, {11:F6}]\n" +
                "[{12:F6}, {13:F6}, {14:F6}, {15:F6}]",
                m00, m01, m02, m03,
                m10, m11, m12, m13,
                m20, m21, m22, m23,
                m30, m31, m32, m33
            );
        }
    }
}
