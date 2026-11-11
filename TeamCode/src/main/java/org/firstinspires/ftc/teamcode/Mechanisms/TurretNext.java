package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

import dev.nextftc.hardware.actuators.NextCRServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.Telemetry;

@Config
public class TurretNext implements Mechanism {

    public static String SERVO_NAME = "storreta2";
    public static String SERVO_NAME1 = "storreta";
    public static String ENCODER_NAME = "encoder";

    public static double FIELD_MIDDLE_Y = 72.0;
    public static double LINE_HYSTERESIS = 2.0;

    public static double HIVE_A_X = 80;
    public static double HIVE_A_Y = 70;

    public static double HIVE_B_X = 70.75;
    public static double HIVE_B_Y = 98;

    public static double TURRET_TEETH = 171.0;
    public static double ENCODER_TEETH = 60.0;
    public static double ENCODER_CPR = 8192.0;

    public static double SERVO_SIGN = -1.0;
    public static double ENCODER_SIGN = -1.0;

    public static double TURRET_MIN_DEG = -180.0;
    public static double TURRET_MAX_DEG = 180.0;
    public static double SAFETY_MARGIN_DEG = 3.0;

    public static double KP = 1;
    public static double KI = .02;
    public static double KD = 0.06;
    public static double MAX_POWER = 1.0;
    public static double DEADBAND = 1.0;
    public static double ON_TARGET_TOLERANCE_DEG = 2.0;


    public static boolean SHOOT_WHILE_MOVING = true;
    public static double FLIGHT_TIME_BASE = 0.25;      // segundos
    public static double FLIGHT_TIME_PER_INCH = 0.004; // segundos por polegada
    public static double VELOCITY_FILTER = 0.3;
    public static double VELOCITY_DEADZONE = 1.0;

    private final NextCRServo servo = new NextCRServo(SERVO_NAME);
    private final NextCRServo servo1 = new NextCRServo(SERVO_NAME1);

    private DcMotorEx encoder;
    private Supplier<Pose> poseSupplier;

    private final SimplePID pid = new SimplePID();

    private boolean autoAimEnabled = true;
    private boolean poseReceived = false;
    private boolean aimingHiveA;

    private double currentAngle;
    private double fieldAngle;
    private double targetAngle;
    private double error;
    private double power;

    private boolean limitBlocked;

    private double velX, velY;
    private double lastX, lastY;
    private long lastVelTimeNs;
    private boolean velInit = false;

    // Debug
    private double aimX, aimY, flightTime;
    private double aimDistance;
    private double radialSpeed;

    public void init(HardwareMap hardwareMap, boolean resetEncoder, Supplier<Pose> poseSupplier) {

        this.poseSupplier = poseSupplier;

        encoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);

        if (resetEncoder) {
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        poseReceived = false;
        velInit = false;
        velX = 0;
        velY = 0;

        servo.setPower(0);
        servo1.setPower(0);
    }

    @Override
    public void periodic() {

        if (encoder == null || poseSupplier == null) {
            return;
        }

        Pose robot = poseSupplier.get();

        if (robot == null) {
            return;
        }

        double robotX = robot.x();
        double robotY = robot.y();
        double robotHeading = Math.toDegrees(robot.heading());

        updateVelocity(robotX, robotY);

        double ticksPerRev = ENCODER_CPR * TURRET_TEETH / ENCODER_TEETH;
        double degreesPerTick = 360.0 / ticksPerRev;

        currentAngle = ENCODER_SIGN * encoder.getCurrentPosition() * degreesPerTick;

        if (!poseReceived) {
            aimingHiveA = robotY > FIELD_MIDDLE_Y;
            poseReceived = true;
        } else if (aimingHiveA && robotY < FIELD_MIDDLE_Y - LINE_HYSTERESIS) {
            aimingHiveA = false;
        } else if (!aimingHiveA && robotY > FIELD_MIDDLE_Y + LINE_HYSTERESIS) {
            aimingHiveA = true;
        }

        double hiveX = aimingHiveA ? HIVE_A_X : HIVE_B_X;
        double hiveY = aimingHiveA ? HIVE_A_Y : HIVE_B_Y;

        // Ponto de mira: o hive "deslocado" pela velocidade do robô.
        // A bola herda a velocidade do robô, então miramos onde o hive
        // estaria em relação à bola durante o voo: alvo = hive - velocidade * tempo.
        aimX = hiveX;
        aimY = hiveY;
        flightTime = 0;

        if (SHOOT_WHILE_MOVING) {
            double speed = Math.hypot(velX, velY);

            if (speed > VELOCITY_DEADZONE) {
                // O tempo de voo depende da distância até o ponto de mira,
                // que depende do tempo -> 3 iterações resolvem bem.
                for (int i = 0; i < 3; i++) {
                    double dist = Math.hypot(aimX - robotX, aimY - robotY);
                    flightTime = FLIGHT_TIME_BASE + dist * FLIGHT_TIME_PER_INCH;
                    aimX = hiveX - velX * flightTime;
                    aimY = hiveY - velY * flightTime;
                }
            }
        }

        // Velocidade radial: quanto o robô se afasta (+) ou se aproxima (-) do hive, em pol/s
        double toRobotX = robotX - hiveX;
        double toRobotY = robotY - hiveY;
        double hiveDist = Math.hypot(toRobotX, toRobotY);
        radialSpeed = (SHOOT_WHILE_MOVING && hiveDist > 1e-3)
                ? (velX * toRobotX + velY * toRobotY) / hiveDist
                : 0.0;

        // Distância até o ponto de mira (já compensado) -> usada pelo shooter
        aimDistance = Math.hypot(aimX - robotX, aimY - robotY);

        fieldAngle = Math.toDegrees(
                Math.atan2(aimY - robotY, aimX - robotX)
        );

        targetAngle = clip(
                wrapAngle(fieldAngle - robotHeading),
                TURRET_MIN_DEG + SAFETY_MARGIN_DEG,
                TURRET_MAX_DEG - SAFETY_MARGIN_DEG
        );

        error = targetAngle - currentAngle;

        if (!autoAimEnabled || Math.abs(error) <= DEADBAND) {
            power = 0;
            pid.reset();
        } else {
            power = clip(pid.calculate(error), -MAX_POWER, MAX_POWER);
        }

        limitBlocked = false;

        if (currentAngle >= TURRET_MAX_DEG && power > 0) {
            power = 0;
            limitBlocked = true;
        }

        if (currentAngle <= TURRET_MIN_DEG && power < 0) {
            power = 0;
            limitBlocked = true;
        }

        servo.setPower(SERVO_SIGN * power);
        servo1.setPower(SERVO_SIGN * power);
    }

    /** Velocidade do robô no campo (pol/s), calculada pela variação da pose + filtro. */
    private void updateVelocity(double x, double y) {
        long now = System.nanoTime();

        if (!velInit) {
            lastX = x;
            lastY = y;
            lastVelTimeNs = now;
            velX = 0;
            velY = 0;
            velInit = true;
            return;
        }

        double dt = (now - lastVelTimeNs) / 1e9;

        if (dt <= 1e-4 || dt > 0.25) {
            lastX = x;
            lastY = y;
            lastVelTimeNs = now;
            return;
        }

        double rawVx = (x - lastX) / dt;
        double rawVy = (y - lastY) / dt;

        velX += VELOCITY_FILTER * (rawVx - velX);
        velY += VELOCITY_FILTER * (rawVy - velY);

        lastX = x;
        lastY = y;
        lastVelTimeNs = now;
    }

    public Command enableAutoAim() {
        return instant(() -> autoAimEnabled = true);
    }

    public Command disableAutoAim() {
        return instant(() -> {
            autoAimEnabled = false;
            pid.reset();
        });
    }

    public Command toggleAutoAim() {
        return instant(() -> {
            autoAimEnabled = !autoAimEnabled;
            pid.reset();
        });
    }

    public Command toggleShootWhileMoving() {
        return instant(() -> SHOOT_WHILE_MOVING = !SHOOT_WHILE_MOVING);
    }

    public void setAutoAimEnabled(boolean enabled) {
        autoAimEnabled = enabled;

        if (!enabled) {
            pid.reset();
        }
    }

    public void setShootWhileMoving(boolean enabled) {
        SHOOT_WHILE_MOVING = enabled;
    }

    public void stop() {
        autoAimEnabled = false;
        power = 0;
        pid.reset();

        servo.setPower(0);
        servo1.setPower(0);
    }

    public boolean isOnTarget() {
        return autoAimEnabled
                && poseReceived
                && Math.abs(error) <= ON_TARGET_TOLERANCE_DEG;
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    public boolean isShootWhileMoving() {
        return SHOOT_WHILE_MOVING;
    }

    public boolean isAimingHiveA() {
        return aimingHiveA;
    }

    /** Distância até o ponto de mira compensado (0 até o primeiro periodic). */
    public double getAimDistance() {
        return aimDistance;
    }

    /** Velocidade (pol/s) com que o robô se afasta (+) ou se aproxima (-) do hive. */
    public double getRadialSpeed() {
        return radialSpeed;
    }

    public double getAngle() {
        return currentAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getError() {
        return error;
    }

    public double getPower() {
        return power;
    }

    public boolean isLimitBlocked() {
        return limitBlocked;
    }

    public void logTelemetry() {
        Telemetry.log("Hive Alvo", aimingHiveA ? "A" : "B");
        Telemetry.log("Field Angle", fieldAngle);
        Telemetry.log("Turret Angle", currentAngle);
        Telemetry.log("Turret Target", targetAngle);
        Telemetry.log("Error", error);
        Telemetry.log("On Target", isOnTarget());
        Telemetry.log("Servo Power", power);
        Telemetry.log("Limit Blocked", limitBlocked);
        Telemetry.log("Auto Aim", autoAimEnabled);
        Telemetry.log("SWM", SHOOT_WHILE_MOVING);
        Telemetry.log("Vel X", velX);
        Telemetry.log("Vel Y", velY);
        Telemetry.log("Flight Time", flightTime);
        Telemetry.log("Aim X", aimX);
        Telemetry.log("Aim Y", aimY);
        Telemetry.log("Aim Dist", aimDistance);
        Telemetry.log("Radial Speed", radialSpeed);
    }

    private static class SimplePID {

        private double integral;
        private double lastError;
        private long lastTimeNs;
        private boolean first = true;

        double calculate(double error) {

            long now = System.nanoTime();
            double dt = first ? 0.0 : (now - lastTimeNs) / 1e9;

            double derivative = 0.0;

            if (!first && dt > 0) {
                integral += error * dt;
                derivative = (error - lastError) / dt;
            }
            first = false;
            lastError = error;
            lastTimeNs = now;

            return KP * error + KI * integral + KD * derivative;
        }
        void reset() {
            integral = 0.0;
            lastError = 0.0;
            first = true;
        }
    }
    private double wrapAngle(double angle) {
        while (angle > 180) {
            angle -= 360;
        }

        while (angle < -180) {
            angle += 360;
        }

        return angle;
    }
    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}