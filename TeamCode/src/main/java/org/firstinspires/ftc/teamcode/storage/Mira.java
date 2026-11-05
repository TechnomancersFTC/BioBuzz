package org.firstinspires.ftc.teamcode.storage;

/** Matemática pura de mira (sem FTC SDK, fácil de testar). Ângulos em graus, distâncias em polegadas. */
public final class Mira {
    private Mira() {}

    /** Normaliza para (-180, 180]. */
    public static double wrapDeg(double a) {
        a = a % 360.0;
        if (a > 180.0) a -= 360.0;
        if (a <= -180.0) a += 360.0;
        return a;
    }

    /** Posição do pivô da torreta no campo (leva em conta o offset em relação ao centro do robô). */
    public static double[] pivotField(double x, double y, double headingRad, double offFwd, double offLeft) {
        double c = Math.cos(headingRad), s = Math.sin(headingRad);
        return new double[] { x + offFwd * c - offLeft * s, y + offFwd * s + offLeft * c };
    }

    /** Ângulo da torreta (relativo à frente do robô, anti-horário +) para apontar ao alvo. Resultado em (-180,180]. */
    public static double desiredAngleDeg(double x, double y, double headingRad,
                                         double offFwd, double offLeft, double tx, double ty) {
        double[] p = pivotField(x, y, headingRad, offFwd, offLeft);
        double bearingDeg = Math.toDegrees(Math.atan2(ty - p[1], tx - p[0]));
        return wrapDeg(bearingDeg - Math.toDegrees(headingRad));
    }

    /**
     * Escolhe entre d, d+360 e d-360 o equivalente alcançável (dentro dos limites) mais perto do ângulo atual.
     * Se nenhum couber nos limites, trava no limite mais próximo.
     */
    public static double reachable(double desired, double current, double min, double max) {
        double best = Double.NaN, bestDist = Double.MAX_VALUE;
        for (int k = -1; k <= 1; k++) {
            double c = desired + 360.0 * k;
            if (c >= min && c <= max) {
                double d = Math.abs(c - current);
                if (d < bestDist) { bestDist = d; best = c; }
            }
        }
        if (!Double.isNaN(best)) return best;
        double clamped = Math.max(min, Math.min(max, desired));
        return clamped;
    }

    /** PD + feedforward de velocidade + atrito estático. Saída: potência em [-max, max] (+ = aumenta o ângulo). */
    public static final class Controller {
        public double kP, kD, kS, kV, maxPower, deadbandDeg;

        public Controller(double kP, double kD, double kS, double kV, double maxPower, double deadbandDeg) {
            this.kP = kP; this.kD = kD; this.kS = kS; this.kV = kV;
            this.maxPower = maxPower; this.deadbandDeg = deadbandDeg;
        }

        /**
         * @param targetDeg      ângulo alvo (alcançável)
         * @param currentDeg     ângulo medido
         * @param measuredRate   velocidade medida (°/s)
         * @param targetRate     velocidade do alvo (°/s) — o quanto o alvo "foge" por causa do robô girar/andar
         */
        public double calculate(double targetDeg, double currentDeg, double measuredRate, double targetRate) {
            double err = targetDeg - currentDeg;
            double out = kP * err - kD * measuredRate + kV * targetRate;
            if (Math.abs(err) > deadbandDeg) out += Math.signum(err) * kS;
            else out *= 0.0;
            return Math.max(-maxPower, Math.min(maxPower, out));
        }
    }
}