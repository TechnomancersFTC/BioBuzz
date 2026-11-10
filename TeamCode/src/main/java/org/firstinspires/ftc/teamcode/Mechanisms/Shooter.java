package org.firstinspires.ftc.teamcode.Mechanisms;



import dev.nextftc.hardware.actuators.NextMotor.Direction;
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

    public static volatile double Fkp = 0.05;
    public static volatile double Fki = 0.006;
    public static volatile double Fkd = 0.0;

    public static volatile double Fks = 0.09;
    public static volatile double Fkv = 0.00148;
    public static volatile double Fka = .9;

    public static volatile double goal = 1400.0;


//    public static double Fkp = 0.0008;
//    public static double Fki = 0.00115;
//    public static double Fkd = 0.0;
//
//    public static double Fks = 0.05;
//    public static double Fkv = 0.0024;
//    public static double Fka = 0.01;
//
//    public static double goal = 1220.0;

    public static volatile double RADIAL_GAIN = 1.0;
    public static volatile double RADIAL_MAX = 200.0;


    private final NextMotor shooter = new NextMotor(
            "shooter",
            Radians.of(RADIANS_PER_TICK)

    );
    {
        shooter.setDirection(NextMotor.Direction.REVERSE);
    }


    {
        shooter.setDirection(Direction.FORWARD);
    }

    private double targetVelocity = 0.0;
    private boolean running = false;

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

    /** Velocidade alvo automática: tabela por distância + compensação radial. */
    private double autoTarget() {
        double speed = autoshoot.flywheelSpeed(distanceSupplier.getAsDouble());

        if (radialSpeedSupplier != null) {
            double extra = RADIAL_GAIN * radialSpeedSupplier.getAsDouble();
            extra = Math.max(-RADIAL_MAX, Math.min(RADIAL_MAX, extra));
            speed += extra;
        }

        return Math.max(0.0, speed);
    }

    @Override
    public void periodic() {

        updateGains();

        // atualiza o alvo em tempo real pela distância até o gol
        if (autoDistance && distanceSupplier != null) {
            targetVelocity = autoTarget();
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
        Telemetry.log("Shooter Power", getPower());

        if (running) {
            applyVelocitySetpoint();
        }
    }

    public void setVelocity(double velocityTicksPerSec) {
        targetVelocity = velocityTicksPerSec;
        running = true;

        updateGains();
        applyVelocitySetpoint();
    }

    /** Velocidade fixa (campo "goal" do Dashboard). */
    public void shoot() {
        autoDistance = false;
        distanceSupplier = null;
        radialSpeedSupplier = null;
        setVelocity(goal);
    }

    /** Liga o shooter e mantém a velocidade atualizada pela distância até o gol. */
    public void shootAuto(DoubleSupplier goalDistance) {
        shootAuto(goalDistance, null);
    }

    /**
     * Igual ao anterior, mas também compensa o movimento do robô.
     * radialSpeed: pol/s, positivo = se afastando do gol.
     */
    public void shootAuto(DoubleSupplier goalDistance, DoubleSupplier radialSpeed) {
        distanceSupplier = goalDistance;
        radialSpeedSupplier = radialSpeed;
        autoDistance = true;
        setVelocity(autoTarget());
    }

    public void stop() {
        running = false;
        autoDistance = false;
        distanceSupplier = null;
        radialSpeedSupplier = null;
        targetVelocity = 0.0;

        shooter.setThrottle(0.0);
    }

    public void testMotor() {
        running = false;
        autoDistance = false;
        distanceSupplier = null;
        radialSpeedSupplier = null;
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

    public boolean isAutoDistance() {
        return autoDistance;
    }
}