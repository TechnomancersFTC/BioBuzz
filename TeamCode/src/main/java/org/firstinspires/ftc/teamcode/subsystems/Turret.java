package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.DEADBAND_DEG;
import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.DEG_PER_TICK;
import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.ENCODER_NAME;
import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.ENCODER_SIGN;
import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.MAX_ANGLE_DEG;
import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.MAX_POWER;
import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.MIN_ANGLE_DEG;
import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.SERVO_NAME;
import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.SERVO_SIGN;
import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.TURRET_OFFSET_FWD;
import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.TURRET_OFFSET_LEFT;
import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.kD;
import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.kP;
import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.kS;
import static org.firstinspires.ftc.teamcode.storage.TorretaConfig.kV;

import org.firstinspires.ftc.teamcode.storage.Mira;

/**
 * Torreta com mira automática. Não depende do NextFTC: basta chamar aimAt(...) a cada loop
 * com a pose do Pedro Pathing (que já usa o Pinpoint).
 */
public class Turret {
    private CRServo servo;
    private DcMotorEx encoder;

    private final Mira.Controller controller =
            new Mira.Controller(kP, kD, kS, kV, MAX_POWER, DEADBAND_DEG);

    private long lastNanos = 0;
    private double lastAngle = 0, measuredRate = 0;
    private double lastDesired = Double.NaN, targetRate = 0;

    private double targetAngle = 0;     // último alvo comandado (já alcançável)
    private double lastPower = 0;

    /**
     * @param resetEncoder true no início do AUTO (torreta fisicamente em 0°, apontando para a frente).
     *                     false no TELEOP depois do auto: o encoder mantém a contagem enquanto o robô estiver ligado.
     */
    public void init(HardwareMap hw, boolean resetEncoder) {
        servo = hw.get(CRServo.class, SERVO_NAME);
        encoder = hw.get(DcMotorEx.class, ENCODER_NAME);
        if (resetEncoder) encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastAngle = getAngleDeg();
        lastNanos = System.nanoTime();
    }

    public double getAngleDeg() {
        return ENCODER_SIGN * encoder.getCurrentPosition() * DEG_PER_TICK;
    }

    /** Aponta para (tx, ty) dado a pose atual do robô (polegadas, radianos). Chame todo loop. */
    public void aimAt(double x, double y, double headingRad, double tx, double ty) {
        long now = System.nanoTime();
        double dt = Math.max((now - lastNanos) * 1e-9, 1e-3);
        lastNanos = now;

        double angle = getAngleDeg();
        double rawRate = (angle - lastAngle) / dt;
        measuredRate += 0.5 * (rawRate - measuredRate); // filtro simples
        lastAngle = angle;

        double desired = Mira.desiredAngleDeg(x, y, headingRad, TURRET_OFFSET_FWD, TURRET_OFFSET_LEFT, tx, ty);

        // velocidade do alvo (inclui a rotação do robô) — derivada do ângulo desejado, tratando a volta em 360°
        if (!Double.isNaN(lastDesired)) {
            double rate = Mira.wrapDeg(desired - lastDesired) / dt;
            targetRate += 0.3 * (rate - targetRate);
        }
        lastDesired = desired;

        targetAngle = Mira.reachable(desired, angle, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        drive(controller.calculate(targetAngle, angle, measuredRate, targetRate));
    }

    /** Controle manual (calibração de sinais / emergência). Respeita os limites. */
    public void setManualPower(double p) {
        lastDesired = Double.NaN; targetRate = 0;
        drive(p);
    }

    public void stop() { drive(0); }

    private void drive(double power) {
        double angle = getAngleDeg();
        if ((angle >= MAX_ANGLE_DEG && power > 0) || (angle <= MIN_ANGLE_DEG && power < 0)) power = 0; // limite suave
        lastPower = power;
        servo.setPower(SERVO_SIGN * power);
    }

    public double getTargetAngleDeg() { return targetAngle; }
    public double getErrorDeg()       { return targetAngle - getAngleDeg(); }
    public double getPower()          { return lastPower; }
    public boolean isLocked(double tolDeg) { return Math.abs(getErrorDeg()) < tolDeg; }
}