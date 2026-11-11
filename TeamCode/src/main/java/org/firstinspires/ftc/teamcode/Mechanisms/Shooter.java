
package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.Telemetry;

import static dev.nextftc.units.Units.Radians;
import static dev.nextftc.units.Units.RadiansPerSecond;

import org.firstinspires.ftc.teamcode.constants.autoshoot;

import java.util.function.DoubleSupplier;

@Config
public class Shooter implements Mechanism {

    private static final double TICKS_PER_REV = 28.0;

    private static final double RADIANS_PER_TICK =
            (2.0 * Math.PI) / TICKS_PER_REV;

    // PID + Feedforward
    public static volatile double Fkp = 0;
    public static volatile double Fki = 0;
    public static volatile double Fkd = 0.0;

    public static volatile double Fks = 0.18;
    public static volatile double Fkv = 0.00148;
    public static volatile double Fka = 0;

    public static volatile double goal = 1400.0;

    // Correção radial
    public static volatile double RADIAL_GAIN = 1.0;
    public static volatile double RADIAL_MAX = 200.0;

    // BANG-BANG
    public static volatile boolean BANG_BANG_ENABLED = true;

    // Erro necessário para entrar no Bang-Bang
    public static volatile double BANG_BANG_ENTER_ERROR = 150.0;

    // Erro para voltar ao PID
    public static volatile double BANG_BANG_EXIT_ERROR = 50.0;

    // Potência durante recuperação
    public static volatile double BANG_BANG_POWER = 1.0;

    private final NextMotor shooter = new NextMotor(
            "shooter",
            Radians.of(RADIANS_PER_TICK)
    );

    private final NextMotor shooter2 = new NextMotor("shooter2");

    {
        shooter.setDirection(NextMotor.Direction.FORWARD);
        shooter2.setDirection(NextMotor.Direction.REVERSE);
    }

    private double targetVelocity = 0.0;

    private boolean running = false;
    private boolean bangBangActive = false;

    private DoubleSupplier distanceSupplier = null;
    private DoubleSupplier radialSpeedSupplier = null;

    private boolean autoDistance = false;

    private void updateGains() {
        shooter.getVelocityConstants()
                .withP(Fkp)
                .withI(Fki)
                .withD(Fkd)
                .withS(Fks)
                .withV(Fkv)
                .withA(Fka);
    }

    private void applyVelocitySetpoint() {
        double targetRadiansPerSecond =
                targetVelocity * RADIANS_PER_TICK;

        shooter.setVelocitySetpoint(
                RadiansPerSecond.of(targetRadiansPerSecond)
        );
    }

    private static double clampThrottle(double value) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            return 0.0;
        }

        return Math.max(-1.0, Math.min(1.0, value));
    }

    /**
     * Motor 2 acompanha o throttle do motor 1.
     */
    private void applyFollower() {
        shooter2.setThrottle(
                clampThrottle(shooter.getThrottle())
        );
    }

    /**
     * Controle híbrido PID + Bang-Bang.
     */
    private void applyHybridControl() {

        double currentVelocity = getVelocity();

        double error = targetVelocity - currentVelocity;

        if (BANG_BANG_ENABLED) {

            if (!bangBangActive &&
                    error >= BANG_BANG_ENTER_ERROR) {

                bangBangActive = true;
            }

            if (bangBangActive &&
                    error <= BANG_BANG_EXIT_ERROR) {

                bangBangActive = false;
            }

        } else {
            bangBangActive = false;
        }

        if (bangBangActive) {

            shooter.setThrottle(
                    clampThrottle(BANG_BANG_POWER)
            );

        } else {

            applyVelocitySetpoint();
        }

        applyFollower();
    }

    private double autoTarget() {

        double speed = autoshoot.flywheelSpeed(
                distanceSupplier.getAsDouble()
        );

        if (radialSpeedSupplier != null) {

            double extra =
                    RADIAL_GAIN * radialSpeedSupplier.getAsDouble();

            extra = Math.max(
                    -RADIAL_MAX,
                    Math.min(RADIAL_MAX, extra)
            );

            speed += extra;
        }

        return Math.max(0.0, speed);
    }

    @Override
    public void periodic() {

        updateGains();

        if (autoDistance && distanceSupplier != null) {
            targetVelocity = autoTarget();
        }

        if (running) {
            applyHybridControl();
        }

        Telemetry.log("Shooter Fkp", Fkp);
        Telemetry.log("Shooter Fki", Fki);
        Telemetry.log("Shooter Fkd", Fkd);

        Telemetry.log("Shooter Fks", Fks);
        Telemetry.log("Shooter Fkv", Fkv);
        Telemetry.log("Shooter Fka", Fka);

        Telemetry.log("Shooter Goal", goal);

        Telemetry.log("Shooter Running", running);
        Telemetry.log("Shooter AutoDist", autoDistance);

        Telemetry.log("Shooter Target", targetVelocity);
        Telemetry.log("Shooter Velocity", getVelocity());
        Telemetry.log("Shooter Error", getVelocityError());

        Telemetry.log("Shooter BangBang", bangBangActive);
        Telemetry.log("Shooter BangBang Enabled", BANG_BANG_ENABLED);

        Telemetry.log("Shooter 1 Power", getPower());
        Telemetry.log("Shooter 2 Power", shooter2.getThrottle());
    }

    public void setVelocity(double velocityTicksPerSec) {

        targetVelocity = velocityTicksPerSec;

        running = true;
        bangBangActive = false;

        updateGains();

        applyVelocitySetpoint();
        applyFollower();
    }

    public void shoot() {

        autoDistance = false;

        distanceSupplier = null;
        radialSpeedSupplier = null;

        setVelocity(goal);
    }

    public void shootAuto(DoubleSupplier goalDistance) {
        shootAuto(goalDistance, null);
    }

    public void shootAuto(
            DoubleSupplier goalDistance,
            DoubleSupplier radialSpeed
    ) {

        distanceSupplier = goalDistance;
        radialSpeedSupplier = radialSpeed;

        autoDistance = true;

        setVelocity(autoTarget());
    }

    public void stop() {

        running = false;
        autoDistance = false;

        bangBangActive = false;

        distanceSupplier = null;
        radialSpeedSupplier = null;

        targetVelocity = 0.0;

        shooter.setThrottle(0.0);
        shooter2.setThrottle(0.0);
    }

    public void testMotor() {

        running = false;
        autoDistance = false;

        bangBangActive = false;

        distanceSupplier = null;
        radialSpeedSupplier = null;

        targetVelocity = 0.0;

        shooter.setThrottle(0.3);
        shooter2.setThrottle(0.3);
    }

    public double getVelocity() {
        return shooter.getEncoderVelocity().getMagnitude()
                / RADIANS_PER_TICK;
    }

    public double getVelocity2() {
        return getVelocity();
    }

    public double getTarget() {
        return targetVelocity;
    }

    public double getVelocityError() {
        return targetVelocity - getVelocity();
    }

    public double getPower() {
        return shooter.getThrottle();
    }

    public boolean isRunning() {
        return running;
    }

    public boolean isAutoDistance() {
        return autoDistance;
    }

    public boolean isBangBangActive() {
        return bangBangActive;
    }
}
