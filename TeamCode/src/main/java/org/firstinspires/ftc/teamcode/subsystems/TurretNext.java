
package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Supplier;

import dev.nextftc.hardware.actuators.NextCRServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.Telemetry;

public class TurretNext implements Mechanism {

    private static final String SERVO_NAME = "storreta2";
    private static final String SERVO_NAME1 = "storreta";
    private static final String ENCODER_NAME = "encoder";

    private static final double FIELD_MIDDLE_Y = 72.0;
    private static final double LINE_HYSTERESIS = 2.0;

    private static final double HIVE_A_X = 84.5;
    private static final double HIVE_A_Y = 80.22;

    private static final double HIVE_B_X = 84.5;
    private static final double HIVE_B_Y = 56.83;

    private static final double TURRET_TEETH = 171.0;
    private static final double ENCODER_TEETH = 60.0;
    private static final double ENCODER_CPR = 8192.0;

    private static final double TICKS_PER_REV =
            ENCODER_CPR * TURRET_TEETH / ENCODER_TEETH;

    private static final double DEG_PER_TICK = 360.0 / TICKS_PER_REV;

    private static final double SERVO_SIGN = -1.0;
    private static final double ENCODER_SIGN = -1.0;

    private static final double TURRET_MIN_DEG = -180.0;
    private static final double TURRET_MAX_DEG = 180.0;

    private static final double SAFETY_MARGIN_DEG = 3.0;

    private static final double KP = 5;
    private static final double KI = 2;
    private static final double KD = 0.0;

    private static final double MAX_POWER = 1;
    private static final double DEADBAND = 1.0;
    private static final double ON_TARGET_TOLERANCE_DEG = 2.0;

    private final NextCRServo servo = new NextCRServo(SERVO_NAME);
    private final NextCRServo servo1 = new NextCRServo(SERVO_NAME1);

    private DcMotorEx encoder;

    private Supplier<Pose> poseSupplier;

    private final SimplePID pid = new SimplePID(KP, KI, KD);

    private boolean autoAimEnabled = true;
    private boolean poseReceived = false;
    private boolean aimingHiveA;

    private double currentAngle;
    private double fieldAngle;
    private double targetAngle;
    private double error;
    private double power;

    private boolean limitBlocked;

    public void init(HardwareMap hardwareMap, boolean resetEncoder, Supplier<Pose> poseSupplier) {

        this.poseSupplier = poseSupplier;

        encoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);

        if (resetEncoder) {
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        poseReceived = false;

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

        currentAngle = ENCODER_SIGN * encoder.getCurrentPosition() * DEG_PER_TICK;

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

        fieldAngle = Math.toDegrees(Math.atan2(hiveY - robotY, hiveX - robotX));

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

    public void setAutoAimEnabled(boolean enabled) {
        autoAimEnabled = enabled;

        if (!enabled) {
            pid.reset();
        }
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

    public boolean isAimingHiveA() {
        return aimingHiveA;
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
        Telemetry.log("Hive Alvo", aimingHiveA ? "A (Y > 72)" : "B (Y < 72)");
        Telemetry.log("Field Angle", fieldAngle);
        Telemetry.log("Turret Angle", currentAngle);
        Telemetry.log("Turret Target", targetAngle);
        Telemetry.log("Error", error);
        Telemetry.log("On Target", isOnTarget());
        Telemetry.log("Servo Power", power);
        Telemetry.log("Limit Blocked", limitBlocked);
    }

    private static class SimplePID {

        private final double kP;
        private final double kI;
        private final double kD;

        private double integral;
        private double lastError;
        private long lastTimeNs;
        private boolean first = true;

        SimplePID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

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

            return kP * error + kI * integral + kD * derivative;
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

