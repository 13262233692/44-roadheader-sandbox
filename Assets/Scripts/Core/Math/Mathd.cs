using System;

namespace RoadheaderSandbox.Core.Math
{
    public static class Mathd
    {
        public const double PI = Math.PI;
        public const double TwoPI = Math.PI * 2.0;
        public const double HalfPI = Math.PI * 0.5;
        public const double Deg2Rad = Math.PI / 180.0;
        public const double Rad2Deg = 180.0 / Math.PI;
        public const double Epsilon = 1e-12;

        public static double Sin(double value) => Math.Sin(value);
        public static double Cos(double value) => Math.Cos(value);
        public static double Tan(double value) => Math.Tan(value);
        public static double Asin(double value) => Math.Asin(Clamp(value, -1.0, 1.0));
        public static double Acos(double value) => Math.Acos(Clamp(value, -1.0, 1.0));
        public static double Atan(double value) => Math.Atan(value);
        public static double Atan2(double y, double x) => Math.Atan2(y, x);

        public static double Sqrt(double value) => Math.Sqrt(value);
        public static double Pow(double x, double y) => Math.Pow(x, y);
        public static double Exp(double value) => Math.Exp(value);
        public static double Log(double value) => Math.Log(value);
        public static double Log10(double value) => Math.Log10(value);

        public static double Abs(double value) => Math.Abs(value);
        public static int Abs(int value) => Math.Abs(value);

        public static double Min(double a, double b) => a < b ? a : b;
        public static double Min(params double[] values)
        {
            if (values == null || values.Length == 0) return 0;
            double result = values[0];
            for (int i = 1; i < values.Length; i++)
                if (values[i] < result) result = values[i];
            return result;
        }

        public static double Max(double a, double b) => a > b ? a : b;
        public static double Max(params double[] values)
        {
            if (values == null || values.Length == 0) return 0;
            double result = values[0];
            for (int i = 1; i < values.Length; i++)
                if (values[i] > result) result = values[i];
            return result;
        }

        public static double Clamp(double value, double min, double max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        public static double Clamp01(double value)
        {
            if (value < 0.0) return 0.0;
            if (value > 1.0) return 1.0;
            return value;
        }

        public static double Lerp(double a, double b, double t)
        {
            return a + (b - a) * Clamp01(t);
        }

        public static double LerpUnclamped(double a, double b, double t)
        {
            return a + (b - a) * t;
        }

        public static double SmoothStep(double a, double b, double t)
        {
            t = Clamp01(t);
            t = t * t * (3.0 - 2.0 * t);
            return LerpUnclamped(a, b, t);
        }

        public static double Sign(double value)
        {
            if (value > 0.0) return 1.0;
            if (value < 0.0) return -1.0;
            return 0.0;
        }

        public static double Floor(double value) => Math.Floor(value);
        public static double Ceiling(double value) => Math.Ceiling(value);
        public static double Round(double value) => Math.Round(value);
        public static double Round(double value, int digits) => Math.Round(value, digits);

        public static double Repeat(double t, double length)
        {
            return t - Floor(t / length) * length;
        }

        public static double PingPong(double t, double length)
        {
            t = Repeat(t, length * 2.0);
            return length - Abs(t - length);
        }

        public static double InverseLerp(double a, double b, double value)
        {
            if (a == b) return 0.0;
            return Clamp01((value - a) / (b - a));
        }

        public static double MoveTowards(double current, double target, double maxDelta)
        {
            if (Abs(target - current) <= maxDelta) return target;
            return current + Sign(target - current) * maxDelta;
        }

        public static double DeltaAngle(double current, double target)
        {
            double delta = Repeat(target - current, 360.0);
            if (delta > 180.0) delta -= 360.0;
            return delta;
        }

        public static double MoveTowardsAngle(double current, double target, double maxDelta)
        {
            double delta = DeltaAngle(current, target);
            if (-maxDelta < delta && delta < maxDelta) return target;
            target = current + delta;
            return MoveTowards(current, target, maxDelta);
        }

        public static double SmoothDamp(double current, double target, ref double currentVelocity, double smoothTime, double maxSpeed, double deltaTime)
        {
            smoothTime = Max(0.0001, smoothTime);
            double omega = 2.0 / smoothTime;
            double x = omega * deltaTime;
            double exp = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x);
            double change = current - target;
            double originalTo = target;
            double maxChange = maxSpeed * smoothTime;
            change = Clamp(change, -maxChange, maxChange);
            target = current - change;
            double temp = (currentVelocity + omega * change) * deltaTime;
            currentVelocity = (currentVelocity - omega * temp) * exp;
            double output = target + (change + temp) * exp;
            if (originalTo - current > 0.0 == output > originalTo)
            {
                output = originalTo;
                currentVelocity = (output - originalTo) / deltaTime;
            }
            return output;
        }

        public static int Factorial(int n)
        {
            if (n < 0) throw new ArgumentException("Factorial is not defined for negative numbers.");
            if (n <= 1) return 1;
            int result = 1;
            for (int i = 2; i <= n; i++) result *= i;
            return result;
        }

        public static double Binomial(int n, int k)
        {
            if (k < 0 || k > n) return 0;
            if (k > n - k) k = n - k;
            double result = 1.0;
            for (int i = 1; i <= k; i++)
            {
                result *= (n - k + i);
                result /= i;
            }
            return result;
        }
    }
}
