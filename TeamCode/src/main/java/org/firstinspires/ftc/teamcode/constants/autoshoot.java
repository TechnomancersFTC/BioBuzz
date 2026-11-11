package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class autoshoot {
    public static double flywheeloffset = 70;
    public static double distoffset = 0;

    public static void SetFlywheelOffset(double offset) {
        flywheeloffset = offset;
    }

    private static final double[] X = {43, 47.30, 55.04, 60, 66, 68.35, 77.83, 80};
    private static final double[] Y = {1000, 1000, 1180, 1100, 1200, 1200, 1200, 1300};
    private static final double[] M = computeSlopes(X, Y);

    public static double flywheelSpeed(double goalDist) {
        double d = goalDist + distoffset;
        double speed = interpolate(d);
        return Math.max(900, Math.min(2500, speed)) + flywheeloffset;    }

    private static double interpolate(double x) {
        int n = X.length;
        if (x <= X[0]) return Y[0];
        if (x >= X[n - 1]) return Y[n - 1];

        int k = 0;
        while (k < n - 2 && x > X[k + 1]) k++;

        double h = X[k + 1] - X[k];
        double t = (x - X[k]) / h;
        double t2 = t * t, t3 = t2 * t;

        double h00 = 2 * t3 - 3 * t2 + 1;
        double h10 = t3 - 2 * t2 + t;
        double h01 = -2 * t3 + 3 * t2;
        double h11 = t3 - t2;

        return h00 * Y[k] + h10 * h * M[k] + h01 * Y[k + 1] + h11 * h * M[k + 1];
    }

    private static double[] computeSlopes(double[] x, double[] y) {
        int n = x.length;
        double[] h = new double[n - 1];
        double[] d = new double[n - 1];
        for (int i = 0; i < n - 1; i++) {
            h[i] = x[i + 1] - x[i];
            d[i] = (y[i + 1] - y[i]) / h[i];
        }

        double[] m = new double[n];

        for (int i = 1; i < n - 1; i++) {
            if (d[i - 1] * d[i] <= 0) {
                m[i] = 0;
            } else {
                double w1 = 2 * h[i] + h[i - 1];
                double w2 = h[i] + 2 * h[i - 1];
                m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i]);
            }
        }

        m[0] = endSlope(h[0], h[1], d[0], d[1]);
        m[n - 1] = endSlope(h[n - 2], h[n - 3], d[n - 2], d[n - 3]);
        return m;
    }

    private static double endSlope(double h0, double h1, double d0, double d1) {
        double m = ((2 * h0 + h1) * d0 - h0 * d1) / (h0 + h1);
        if (Math.signum(m) != Math.signum(d0)) return 0;
        if (Math.signum(d0) != Math.signum(d1) && Math.abs(m) > 3 * Math.abs(d0)) return 3 * d0;
        return m;
    }
}