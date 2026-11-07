
package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

import static dev.nextftc.units.Units.Radians;
import static dev.nextftc.units.Units.RadiansPerSecond;

@Configurable
public class Shooter implements Mechanism {

    private static final double TICKS_PER_REV = 28.0;

    private static final double RADIANS_PER_TICK =
            (2.0 * Math.PI) / TICKS_PER_REV;

    public static double Fkp = 0.0;
    public static double Fki = 0.0;
    public static double Fkd = 0.0;

    public static double Fks = 0.0;
    public static double Fkv = 0.0;
    public static double Fka = 0.0;

    public static double goal = 1400.0;

    private final NextMotor shooter = new NextMotor(
            "shooter",
            Radians.of(RADIANS_PER_TICK)
    );

    private double targetVelocity = 0.0;
    private boolean running = false;

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

    @Override
    public void periodic() {
        if (!running) {
            return;
        }

        updateGains();

        applyVelocitySetpoint();
    }

    public void setVelocity(double velocityTicksPerSec) {
        targetVelocity = velocityTicksPerSec;
        running = true;

        updateGains();
        applyVelocitySetpoint();
    }

    public void shoot() {
        setVelocity(goal);
    }

    public void stop() {
        running = false;
        targetVelocity = 0.0;
        shooter.setThrottle(0.0);
    }

    public void testMotor() {
        running = false;
        targetVelocity = 0.0;
        shooter.setThrottle(0.3);
    }

    public double getVelocity() {
        return shooter.getEncoderVelocity().getMagnitude()
                / RADIANS_PER_TICK;
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
}
