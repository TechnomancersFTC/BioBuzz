package org.firstinspires.ftc.teamcode.Mechanisms;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

import static dev.nextftc.units.Units.Radians;
import static dev.nextftc.units.Units.RadiansPerSecond;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Shooter implements Mechanism {

    public static final Shooter INSTANCE = new Shooter();

    public static double Fkp = 0;
    public static double Fki = 0;
    public static double Fkd = 0;

    public static double Fks = 0;
    public static double Fkv = 0;
    public static double Fka = 0;

    public static double goal = 1400;

    private final NextMotor shooter = new NextMotor(
            "shooter",
            Radians.of(1.0)
    );

    private double targetVelocity = 0;
    private boolean running = false;

    public void initialize() {
        updateGains();
        shooter.setThrottle(0);
    }

    @Override
    public void periodic() {

        if (!running) {
            shooter.setThrottle(0);
            return;
        }

        updateGains();
        shooter.update();
    }

    public void updateGains() {
        shooter.getVelocityConstants()
                .withP(Fkp)
                .withI(Fki)
                .withD(Fkd)
                .withS(Fks)
                .withV(Fkv)
                .withA(Fka);
    }

    public void setVelocity(double velocity) {
        targetVelocity = velocity;
        running = true;

        updateGains();

        shooter.setVelocitySetpoint(
                RadiansPerSecond.of(velocity)
        );
    }

    public void shoot() {
        setVelocity(goal);
    }

    public void stop() {
        running = false;
        targetVelocity = 0;
        shooter.setThrottle(0);
    }

    public double getTarget() {
        return targetVelocity;
    }

    public double getVelocity() {
        return shooter.getEncoderVelocity().getMagnitude();
    }

    public double getPower() {
        return shooter.getThrottle();
    }

    public double getVelocityError() {
        return targetVelocity - getVelocity();
    }

    public boolean isRunning() {
        return running;
    }
}