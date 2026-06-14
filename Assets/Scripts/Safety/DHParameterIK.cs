using System;
using System.Collections.Generic;
using RoadheaderSandbox.Core.Math;
using UnityEngine;

namespace RoadheaderSandbox.Safety
{
    [Serializable]
    public struct DHParameter
    {
        public double theta;
        public double d;
        public double a;
        public double alpha;
        public double jointMin;
        public double jointMax;
        public double jointVelocityMax;
        public bool isRevolute;
        public string jointName;

        public DHParameter(string name, double theta0, double d0, double a0, double alpha0,
                           double min, double max, double velMax, bool revolute = true)
        {
            jointName = name;
            theta = theta0;
            d = d0;
            a = a0;
            alpha = alpha0;
            jointMin = min;
            jointMax = max;
            jointVelocityMax = velMax;
            isRevolute = revolute;
        }

        public Matrix4x4d GetTransform(double jointValue)
        {
            double t = isRevolute ? theta + jointValue : theta;
            double dd = isRevolute ? d : d + jointValue;
            double ct = Mathd.Cos(t);
            double st = Mathd.Sin(t);
            double ca = Mathd.Cos(alpha);
            double sa = Mathd.Sin(alpha);

            Matrix4x4d m = Matrix4x4d.Identity;
            m.m00 = ct; m.m01 = -st * ca; m.m02 = st * sa; m.m03 = a * ct;
            m.m10 = st; m.m11 = ct * ca; m.m12 = -ct * sa; m.m13 = a * st;
            m.m20 = 0; m.m21 = sa; m.m22 = ca; m.m23 = dd;
            m.m30 = 0; m.m31 = 0; m.m32 = 0; m.m33 = 1;
            return m;
        }

        public Matrix4x4d GetTransformDerivative(double jointValue)
        {
            double t = isRevolute ? theta + jointValue : theta;
            double dd = isRevolute ? d : d + jointValue;
            double ct = Mathd.Cos(t);
            double st = Mathd.Sin(t);
            double ca = Mathd.Cos(alpha);
            double sa = Mathd.Sin(alpha);

            Matrix4x4d dm = Matrix4x4d.Zero;
            if (isRevolute)
            {
                dm.m00 = -st; dm.m01 = -ct * ca; dm.m02 = ct * sa; dm.m03 = -a * st;
                dm.m10 = ct; dm.m11 = -st * ca; dm.m12 = st * sa; dm.m13 = a * ct;
            }
            else
            {
                dm.m23 = 1.0;
            }
            return dm;
        }
    }

    [Serializable]
    public class IKSolution
    {
        public double[] jointAngles;
        public double[] jointVelocities;
        public Vector3d tcpPosition;
        public Quaterniond tcpRotation;
        public Vector3d tcpVelocity;
        public Vector3d tcpAngularVelocity;
        public bool isReachable;
        public double positionError;
        public int iterations;
        public double solveTimeMs;

        public IKSolution(int dof)
        {
            jointAngles = new double[dof];
            jointVelocities = new double[dof];
            tcpPosition = Vector3d.Zero;
            tcpRotation = Quaterniond.Identity;
            tcpVelocity = Vector3d.Zero;
            tcpAngularVelocity = Vector3d.Zero;
        }

        public void ClampJointLimits(DHParameter[] parameters)
        {
            for (int i = 0; i < parameters.Length && i < jointAngles.Length; i++)
            {
                jointAngles[i] = Mathd.Clamp(jointAngles[i], parameters[i].jointMin, parameters[i].jointMax);
                double maxV = parameters[i].jointVelocityMax;
                jointVelocities[i] = Mathd.Clamp(jointVelocities[i], -maxV, maxV);
            }
        }
    }

    [Serializable]
    public class DHParameterIK : MonoBehaviour
    {
        [Header("D-H 参数表（悬臂式掘进机6轴标准）")]
        public DHParameter[] parameters = new DHParameter[]
        {
            new DHParameter("回转台Yaw",   0, 0.6, 0,    0,              -45, 45, 30, true),
            new DHParameter("大臂Pitch",   -90, 0, 0.4,  -90,            -10, 85, 25, true),
            new DHParameter("小臂Pitch",   0,   0, 2.8,  0,              -60, 30, 35, true),
            new DHParameter("伸缩缸",      0,   0, 1.2,  0,              0,   1.0, 0.5, false),
            new DHParameter("截割头Roll",  0,   0, 0.3,  0,              -180,180, 150,true),
            new DHParameter("截割头Spin",  0,   0, 0,    0,              -500,500, 300,true),
        };

        [Header("IK求解配置")]
        public int maxIterations = 100;
        public double positionTolerance = 0.001;
        public double orientationTolerance = 0.001;
        public double dampingCoefficient = 0.05;
        public double stepSize = 0.5;
        public bool useDampedLeastSquares = true;

        [Header("D-H测量基准")]
        public Vector3d baseOffset = Vector3d.Zero;
        public Quaterniond baseRotation = Quaterniond.Identity;
        public Vector3d tcpOffset = new Vector3d(0, 0, 0.4);

        [Header("诊断")]
        public int lastSolveIterations;
        public double lastSolveTimeMs;
        public double lastPositionError;
        public double lastOrientationError;

        public event Action<IKSolution> OnIKSolved;
        public event Action<string> OnIKFailed;

        public int DOF => parameters.Length;

        public IKSolution SolveFK(double[] jointValues)
        {
            IKSolution sol = new IKSolution(DOF);
            jointValues.CopyTo(sol.jointAngles, 0);

            Matrix4x4d accum = Matrix4x4d.TRS(baseOffset, baseRotation, Vector3d.One);
            for (int i = 0; i < DOF; i++)
            {
                accum = accum * parameters[i].GetTransform(jointValues[i]);
            }

            Matrix4x4d tcpMat = Matrix4x4d.Translate(tcpOffset);
            accum = accum * tcpMat;

            sol.tcpPosition = new Vector3d(accum.m03, accum.m13, accum.m23);
            sol.tcpRotation = accum.ExtractRotation();
            sol.isReachable = true;
            sol.positionError = 0;
            sol.iterations = 0;
            return sol;
        }

        public IKSolution SolveIK(Vector3d targetPosition, Quaterniond targetRotation, double[] seedJoints = null)
        {
            double timeStart = (double)(DateTime.Now.Ticks / 10000.0);
            IKSolution sol = new IKSolution(DOF);

            double[] q = new double[DOF];
            if (seedJoints != null && seedJoints.Length == DOF)
                seedJoints.CopyTo(q, 0);
            else
            {
                for (int i = 0; i < DOF; i++)
                    q[i] = (parameters[i].jointMin + parameters[i].jointMax) * 0.5;
            }

            int iter = 0;
            bool converged = false;
            double bestError = double.MaxValue;
            double[] bestQ = (double[])q.Clone();

            for (iter = 0; iter < maxIterations; iter++)
            {
                IKSolution fk = SolveFK(q);
                Vector3d posErr = targetPosition - fk.tcpPosition;
                Vector3d rotErr = Quaterniond.AngleAxisDelta(fk.tcpRotation, targetRotation);
                double posError = posErr.Magnitude;
                double rotError = rotErr.Magnitude;

                double totalErr = posError + rotError * 0.5;
                if (totalErr < bestError)
                {
                    bestError = totalErr;
                    q.CopyTo(bestQ, 0);
                }

                if (posError < positionTolerance && rotError < orientationTolerance)
                {
                    converged = true;
                    break;
                }

                Matrix6x6d J = ComputeJacobian(q);
                Vector6d error = new Vector6d(posErr, rotErr);
                Vector6d dq;

                if (useDampedLeastSquares)
                {
                    Matrix6x6d JtJ = J.TransposeMultiply(J);
                    Matrix6x6d reg = JtJ + Matrix6x6d.Scale(dampingCoefficient * dampingCoefficient);
                    Vector6d Jte = J.TransposeMultiply(error);
                    dq = reg.Solve(Jte) * stepSize;
                }
                else
                {
                    Matrix6x6d pinv = J.PseudoInverse(dampingCoefficient);
                    dq = pinv * error * stepSize;
                }

                for (int i = 0; i < DOF; i++)
                {
                    q[i] += dq[i];
                    q[i] = Mathd.Clamp(q[i], parameters[i].jointMin * 0.95, parameters[i].jointMax * 0.95);
                }
            }

            if (!converged)
                bestQ.CopyTo(q, 0);

            IKSolution finalFk = SolveFK(q);
            q.CopyTo(sol.jointAngles, 0);
            sol.tcpPosition = finalFk.tcpPosition;
            sol.tcpRotation = finalFk.tcpRotation;
            sol.positionError = (targetPosition - finalFk.tcpPosition).Magnitude;
            sol.isReachable = converged || bestError < 0.1;
            sol.iterations = iter;
            sol.solveTimeMs = (double)(DateTime.Now.Ticks / 10000.0) - timeStart;

            lastSolveIterations = iter;
            lastSolveTimeMs = sol.solveTimeMs;
            lastPositionError = sol.positionError;
            lastOrientationError = (Quaterniond.AngleAxisDelta(finalFk.tcpRotation, targetRotation)).Magnitude;

            if (sol.isReachable)
                OnIKSolved?.Invoke(sol);
            else
                OnIKFailed?.Invoke($"IK未收敛: posErr={lastPositionError:F4}m iter={iter}");

            return sol;
        }

        public Matrix6x6d ComputeJacobian(double[] jointValues)
        {
            Matrix6x6d J = Matrix6x6d.Zero;
            Matrix4x4d[] transformations = new Matrix4x4d[DOF + 1];
            transformations[0] = Matrix4x4d.TRS(baseOffset, baseRotation, Vector3d.One);

            for (int i = 0; i < DOF; i++)
                transformations[i + 1] = transformations[i] * parameters[i].GetTransform(jointValues[i]);

            Matrix4x4d tcpMat = Matrix4x4d.Translate(tcpOffset);
            transformations[DOF] = transformations[DOF] * tcpMat;

            Vector3d pe = new Vector3d(transformations[DOF].m03, transformations[DOF].m13, transformations[DOF].m23);

            for (int i = 0; i < DOF; i++)
            {
                Matrix4x4d Tinv = transformations[i].Inverse();
                Matrix4x4d dTi = parameters[i].GetTransformDerivative(jointValues[i]);
                Matrix4x4d delta = transformations[i] * dTi * Tinv;
                transformations[i].Dispose();

                Vector3d dTrans = new Vector3d(delta.m03, delta.m13, delta.m23);
                Vector3d skew = new Vector3d(delta.m21, delta.m02, delta.m10);

                J.SetColumn(i, dTrans, skew);
            }

            for (int i = 0; i <= DOF; i++)
                transformations[i].Dispose();

            return J;
        }

        public Vector3d[] ComputeJointTrajectory(Vector3d targetPosition, Quaterniond targetRotation,
                                                  int waypointCount = 20)
        {
            List<Vector3d> trajectory = new List<Vector3d>();
            IKSolution cur = SolveFK(new double[DOF]);

            for (int w = 0; w <= waypointCount; w++)
            {
                double t = (double)w / waypointCount;
                Vector3d interpPos = Vector3d.Lerp(cur.tcpPosition, targetPosition, t);
                Quaterniond interpRot = Quaterniond.Slerp(cur.tcpRotation, targetRotation, t);
                IKSolution step = SolveIK(interpPos, interpRot, (trajectory.Count > 0) ? null : null);
                if (step.isReachable)
                    trajectory.Add(step.tcpPosition);
            }
            return trajectory.ToArray();
        }

        public double ComputePathLength(Vector3d[] waypoints)
        {
            double len = 0;
            for (int i = 1; i < waypoints.Length; i++)
                len += (waypoints[i] - waypoints[i - 1]).Magnitude;
            return len;
        }
    }

    [Serializable]
    public struct Vector6d
    {
        public double v0, v1, v2, v3, v4, v5;

        public Vector6d(double a, double b, double c, double d, double e, double f)
        { v0 = a; v1 = b; v2 = c; v3 = d; v4 = e; v5 = f; }

        public Vector6d(Vector3d linear, Vector3d angular)
        { v0 = linear.x; v1 = linear.y; v2 = linear.z; v3 = angular.x; v4 = angular.y; v5 = angular.z; }

        public double this[int i]
        {
            get { switch (i) { case 0: return v0; case 1: return v1; case 2: return v2; case 3: return v3; case 4: return v4; default: return v5; } }
            set { switch (i) { case 0: v0 = value; break; case 1: v1 = value; break; case 2: v2 = value; break; case 3: v3 = value; break; case 4: v4 = value; break; default: v5 = value; break; } }
        }

        public static Vector6d operator +(Vector6d a, Vector6d b)
        { return new Vector6d(a.v0 + b.v0, a.v1 + b.v1, a.v2 + b.v2, a.v3 + b.v3, a.v4 + b.v4, a.v5 + b.v5); }

        public static Vector6d operator *(Vector6d v, double s)
        { return new Vector6d(v.v0 * s, v.v1 * s, v.v2 * s, v.v3 * s, v.v4 * s, v.v5 * s); }

        public Vector3d Linear => new Vector3d(v0, v1, v2);
        public Vector3d Angular => new Vector3d(v3, v4, v5);
        public double Magnitude => Mathd.Sqrt(v0 * v0 + v1 * v1 + v2 * v2 + v3 * v3 + v4 * v4 + v5 * v5);
    }

    [Serializable]
    public struct Matrix6x6d : IDisposable
    {
        private double[] _data;

        public Matrix6x6d(bool identity = false)
        {
            _data = new double[36];
            if (identity) for (int i = 0; i < 6; i++) _data[i * 7] = 1.0;
        }

        public static Matrix6x6d Zero => new Matrix6x6d(false);
        public static Matrix6x6d Identity => new Matrix6x6d(true);

        public double this[int r, int c]
        {
            get { return _data[r * 6 + c]; }
            set { _data[r * 6 + c] = value; }
        }

        public static Matrix6x6d Scale(double s)
        {
            Matrix6x6d m = new Matrix6x6d();
            for (int i = 0; i < 6; i++) m[i, i] = s;
            return m;
        }

        public void SetColumn(int c, Vector3d linear, Vector3d angular)
        {
            _data[0 * 6 + c] = linear.x;
            _data[1 * 6 + c] = linear.y;
            _data[2 * 6 + c] = linear.z;
            _data[3 * 6 + c] = angular.x;
            _data[4 * 6 + c] = angular.y;
            _data[5 * 6 + c] = angular.z;
        }

        public Vector6d Multiply(Vector6d v)
        {
            Vector6d r = new Vector6d();
            for (int row = 0; row < 6; row++)
            {
                double sum = 0;
                for (int col = 0; col < 6; col++)
                    sum += _data[row * 6 + col] * v[col];
                r[row] = sum;
            }
            return r;
        }

        public static Vector6d operator *(Matrix6x6d m, Vector6d v) => m.Multiply(v);

        public Matrix6x6d Transpose()
        {
            Matrix6x6d t = new Matrix6x6d();
            for (int r = 0; r < 6; r++)
                for (int c = 0; c < 6; c++)
                    t[c, r] = _data[r * 6 + c];
            return t;
        }

        public Matrix6x6d TransposeMultiply(Matrix6x6d other)
        {
            Matrix6x6d r = new Matrix6x6d();
            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 6; j++)
                {
                    double s = 0;
                    for (int k = 0; k < 6; k++)
                        s += _data[k * 6 + i] * other._data[k * 6 + j];
                    r[i, j] = s;
                }
            return r;
        }

        public Vector6d TransposeMultiply(Vector6d v)
        {
            Vector6d r = new Vector6d();
            for (int col = 0; col < 6; col++)
            {
                double s = 0;
                for (int row = 0; row < 6; row++)
                    s += _data[row * 6 + col] * v[row];
                r[col] = s;
            }
            return r;
        }

        public Vector6d Solve(Vector6d b)
        {
            double[,] a = new double[6, 7];
            for (int r = 0; r < 6; r++)
            {
                for (int c = 0; c < 6; c++) a[r, c] = _data[r * 6 + c];
                a[r, 6] = b[r];
            }

            for (int p = 0; p < 6; p++)
            {
                int maxRow = p;
                double maxVal = Mathd.Abs(a[p, p]);
                for (int r = p + 1; r < 6; r++)
                {
                    double v = Mathd.Abs(a[r, p]);
                    if (v > maxVal) { maxVal = v; maxRow = r; }
                }
                if (maxRow != p)
                {
                    for (int c = p; c < 7; c++)
                    {
                        double tmp = a[p, c]; a[p, c] = a[maxRow, c]; a[maxRow, c] = tmp;
                    }
                }

                double pivot = a[p, p];
                if (Mathd.Abs(pivot) < 1e-20) pivot = 1e-20;
                for (int c = p; c < 7; c++) a[p, c] /= pivot;

                for (int r = 0; r < 6; r++)
                {
                    if (r == p) continue;
                    double f = a[r, p];
                    for (int c = p; c < 7; c++)
                        a[r, c] -= f * a[p, c];
                }
            }

            Vector6d x = new Vector6d();
            for (int i = 0; i < 6; i++) x[i] = a[i, 6];
            return x;
        }

        public Matrix6x6d PseudoInverse(double damping = 0.01)
        {
            Matrix6x6d Jt = Transpose();
            Matrix6x6d JtJ = new Matrix6x6d();
            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 6; j++)
                {
                    double s = 0;
                    for (int k = 0; k < 6; k++) s += Jt[i, k] * _data[k * 6 + j];
                    JtJ[i, j] = s;
                }
            for (int i = 0; i < 6; i++) JtJ[i, i] += damping * damping;

            Matrix6x6d invJtJ = JtJ.Inverse3x3Safe();
            Matrix6x6d pinv = new Matrix6x6d();
            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 6; j++)
                {
                    double s = 0;
                    for (int k = 0; k < 6; k++) s += invJtJ[i, k] * Jt[k, j];
                    pinv[i, j] = s;
                }
            return pinv;
        }

        public Matrix6x6d Inverse3x3Safe()
        {
            Matrix6x6d aug = new Matrix6x6d();
            double[,] a = new double[6, 12];
            for (int r = 0; r < 6; r++)
            {
                for (int c = 0; c < 6; c++) a[r, c] = _data[r * 6 + c];
                a[r, 6 + r] = 1.0;
            }

            for (int p = 0; p < 6; p++)
            {
                int maxRow = p;
                double maxVal = Mathd.Abs(a[p, p]);
                for (int r = p + 1; r < 6; r++)
                {
                    double v = Mathd.Abs(a[r, p]);
                    if (v > maxVal) { maxVal = v; maxRow = r; }
                }
                if (maxRow != p)
                {
                    for (int c = p; c < 12; c++)
                    {
                        double tmp = a[p, c]; a[p, c] = a[maxRow, c]; a[maxRow, c] = tmp;
                    }
                }
                double piv = a[p, p];
                if (Mathd.Abs(piv) < 1e-20) piv = 1e-20;
                for (int c = p; c < 12; c++) a[p, c] /= piv;
                for (int r = 0; r < 6; r++)
                {
                    if (r == p) continue;
                    double f = a[r, p];
                    for (int c = p; c < 12; c++)
                        a[r, c] -= f * a[p, c];
                }
            }

            for (int r = 0; r < 6; r++)
                for (int c = 0; c < 6; c++)
                    aug._data[r * 6 + c] = a[r, 6 + c];

            return aug;
        }

        public void Dispose() { _data = null; }
    }
}
