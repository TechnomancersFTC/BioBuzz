package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Mecanismo da torreta com mira automática nas Hives.
 *
 * Não é um OpMode e não chama follower.update(): quem é dono do Follower
 * (TeleOp ou Autônomo) atualiza a odometria e passa a pose para update().
 *
 * Uso típico (no loop do OpMode):
 *
 *   follower.update();
 *   torreta.update(follower.pose());
 *
 * Importante: update() precisa rodar em cada ciclo do loop, inclusive
 * enquanto o autônomo espera algo. Evite sleep()/while bloqueantes.
 *
 * Mira em movimento: o controle é um P mais um feedforward de velocidade
 * (KV). O P sozinho só gera potência quando já existe erro, então com o
 * robô andando/girando a torreta fica sempre atrasada. O feedforward mede
 * o quanto o alvo está girando (graus/s) e já aplica essa potência.
 */
public class Torreta {

    // Nomes no hardwareMap
    private static final String SERVOb_NAME = "storreta2";
    private static final String SERVOa_NAME = "storreta";
    private static final String ENCODER_NAME = "encoder";

    // Linha do meio do campo (no visualizer do Pedro o centro é (72, 72))
    private static final double FIELD_MIDDLE_Y = 72.0;

    // Histerese da linha (polegadas): só troca de Hive depois de passar
    // da linha por essa distância, para não ficar alternando em cima dela.
    // Use 0 para trocar exatamente em Y = 72.
    private static final double LINE_HYSTERESIS = 2.0;

    // Hive A: a que começa levantada (lado Y > 72)
    private static final double HIVE_A_X = 84.5;
    private static final double HIVE_A_Y = 80.22;

    // Hive B: a oposta (lado Y < 72)
    private static final double HIVE_B_X = 84.5;
    private static final double HIVE_B_Y = 56.83;

    // Engrenagens
    private static final double TURRET_TEETH = 171.0;
    private static final double ENCODER_TEETH = 60.0;
    private static final double ENCODER_CPR = 8192.0;

    private static final double TICKS_PER_REV =
            ENCODER_CPR * TURRET_TEETH / ENCODER_TEETH;

    private static final double DEG_PER_TICK =
            360.0 / TICKS_PER_REV;

    // Sinais (inverter os dois juntos)
    private static final double SERVO_SIGN = -1.0;
    private static final double ENCODER_SIGN = -1.0;

    // Limites mecânicos da torreta (graus, relativos à frente do robô)
    private static final double TURRET_MIN_DEG = -180.0;
    private static final double TURRET_MAX_DEG = 180.0;

    // Margem de segurança: o alvo nunca passa de (limite - margem),
    // para o overshoot do controle não bater no limite real.
    // Use 0 se quiser exatamente +-180.
    private static final double SAFETY_MARGIN_DEG = 3.0;

    // Controle proporcional
    private static final double KP = 0.04;
    private static final double MAX_POWER = 0.5;   // teto do termo P
    private static final double DEADBAND = 1.0;

    // Feedforward de velocidade do alvo
    // KV = potência por (grau/s). Aproximadamente 1 / (velocidade da torreta
    // em graus/s com potência 1.0). Como ajustar: veja "Turret Speed" na
    // telemetria durante um giro longo (erro grande, P saturado em MAX_POWER):
    //   KV = MAX_POWER / velocidade medida
    // KV = 0 desliga o feedforward.
    private static final double KV = 0.5;

    // Suavização da velocidade medida (0 a 1). Maior = reage mais rápido,
    // mas passa mais ruído da odometria.
    private static final double RATE_FILTER = 1;

    // Velocidades do alvo abaixo disso (graus/s) são tratadas como zero
    private static final double RATE_DEADZONE_DEG_S = 2.0;

    // Teto da potência total (P + feedforward)
    private static final double MAX_TOTAL_POWER = 1.0;

    // Erro (graus) abaixo do qual a torreta conta como "apontada"
    private static final double ON_TARGET_TOLERANCE_DEG = 2.0;

    // Hardware
    private final CRServo servoA,servoB;
    private final DcMotorEx encoder;

    // Estado
    private boolean autoAimEnabled = true;
    private boolean poseReceived = false;
    private boolean aimingHiveA;

    private double currentAngle;
    private double fieldAngle;
    private double targetAngle;
    private double error;
    private double power;
    private boolean limitBlocked;

    // Estado do feedforward
    private boolean hasLast = false;
    private long lastTimeNanos;
    private double lastRelativeAngle;
    private double lastCurrentAngle;
    private double targetRate;     // graus/s, filtrado
    private double turretSpeed;    // graus/s, filtrado (só telemetria)
    private double ffPower;

    /**
     * @param resetEncoder true  = zera o encoder agora (a torreta DEVE estar
     *                            apontando para a frente do robô). Use no
     *                            início do autônomo.
     *                     false = mantém a contagem atual. Use no TeleOp que
     *                            vem depois do autônomo, para a torreta não
     *                            perder a referência de onde está.
     */
    public Torreta(HardwareMap hardwareMap, boolean resetEncoder) {

        servoA = hardwareMap.get(CRServo.class, SERVOa_NAME);
        servoB = hardwareMap.get(CRServo.class, SERVOb_NAME);
        encoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);

        if (resetEncoder) {
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoA.setPower(0);
        servoB.setPower(0);
    }

    /** Liga/desliga a mira automática. Desligada, a torreta fica parada. */
    public void setAutoAimEnabled(boolean enabled) {
        autoAimEnabled = enabled;
    }

    /** Roda um ciclo de controle. Chame em todos os loop, depois de follower.update(). */
    public void update(Pose robot) {

        double robotX = robot.x();
        double robotY = robot.y();
        double robotHeading = Math.toDegrees(robot.heading());

        // Tempo desde o ciclo anterior
        long now = System.nanoTime();
        double dt = (now - lastTimeNanos) / 1e9;
        lastTimeNanos = now;

        // Ângulo atual da torreta (0 = frente do robô, sem wrap)
        currentAngle =
                ENCODER_SIGN
                        * encoder.getCurrentPosition()
                        * DEG_PER_TICK;

        // Escolhe a Hive pela posição do robô em relação à linha do meio.
        // No primeiro ciclo, começa na Hive do lado em que o robô está.
        boolean previousAimingHiveA = aimingHiveA;

        if (!poseReceived) {
            aimingHiveA = robotY > FIELD_MIDDLE_Y;
            poseReceived = true;
        } else if (aimingHiveA && robotY < FIELD_MIDDLE_Y - LINE_HYSTERESIS) {
            aimingHiveA = false;
        } else if (!aimingHiveA && robotY > FIELD_MIDDLE_Y + LINE_HYSTERESIS) {
            aimingHiveA = true;
        }

        boolean hiveChanged = aimingHiveA != previousAimingHiveA;

        double hiveX = aimingHiveA ? HIVE_A_X : HIVE_B_X;
        double hiveY = aimingHiveA ? HIVE_A_Y : HIVE_B_Y;

        // Ângulo absoluto da Hive alvo em relação ao campo
        fieldAngle = Math.toDegrees(
                Math.atan2(
                        hiveY - robotY,
                        hiveX - robotX
                )
        );

        // Ângulo da Hive relativo à frente do robô (-180 a +180)
        double relativeAngle = wrapAngle(fieldAngle - robotHeading);

        // Mantém o alvo dentro da faixa mecânica
        targetAngle = clip(
                relativeAngle,
                TURRET_MIN_DEG + SAFETY_MARGIN_DEG,
                TURRET_MAX_DEG - SAFETY_MARGIN_DEG
        );

        boolean targetClipped = Math.abs(relativeAngle - targetAngle) > 1e-9;

        // Velocidade com que o alvo gira em relação ao robô (graus/s).
        // Usa o ângulo relativo ANTES do clip e com wrap no delta, então
        // a emenda em +-180 não gera pico. Na troca de Hive o ângulo pula,
        // então esse ciclo é ignorado e mantém a velocidade anterior.
        boolean validDt = dt > 0.001 && dt < 0.25;

        if (hasLast && validDt && !hiveChanged) {
            double rawTargetRate =
                    wrapAngle(relativeAngle - lastRelativeAngle) / dt;
            double rawTurretSpeed = (currentAngle - lastCurrentAngle) / dt;

            targetRate += RATE_FILTER * (rawTargetRate - targetRate);
            turretSpeed += RATE_FILTER * (rawTurretSpeed - turretSpeed);
        } else if (!hasLast || !validDt) {
            targetRate = 0;
            turretSpeed = 0;
        }

        lastRelativeAngle = relativeAngle;
        lastCurrentAngle = currentAngle;
        hasLast = true;

        // Erro no referencial da torreta e SEM wrap:
        // a torreta nunca cruza +-180 (os cabos não enrolam)
        error = targetAngle - currentAngle;

        // Feedforward: já aplica a potência necessária para acompanhar o
        // alvo. Desligado quando o alvo está preso num limite mecânico.
        ffPower = 0;

        if (autoAimEnabled
                && !targetClipped
                && Math.abs(targetRate) > RATE_DEADZONE_DEG_S) {
            ffPower = KV * targetRate;
        }

        // Controle: P + feedforward
        if (!autoAimEnabled) {
            power = 0;
        } else if (Math.abs(error) <= DEADBAND) {
            // Dentro da deadband o P zera, mas o feedforward continua
            // acompanhando o alvo (sem isso a torreta anda aos soquinhos)
            power = clip(ffPower, -MAX_TOTAL_POWER, MAX_TOTAL_POWER);
        } else {
            power = clip(
                    clip(KP * error, -MAX_POWER, MAX_POWER) + ffPower,
                    -MAX_TOTAL_POWER,
                    MAX_TOTAL_POWER
            );
        }

        // Trava de segurança nos limites mecânicos
        limitBlocked = false;

        if (currentAngle >= TURRET_MAX_DEG && power > 0) {
            power = 0;
            limitBlocked = true;
        }

        if (currentAngle <= TURRET_MIN_DEG && power < 0) {
            power = 0;
            limitBlocked = true;
        }

        servoA.setPower(SERVO_SIGN * power);
        servoB.setPower(SERVO_SIGN * power);
    }

    /** Para a torreta. Chame no stop() do OpMode. */
    public void stop() {
        servoA.setPower(0);
        servoB.setPower(0);
    }

    // ---------------- Estado (para o autônomo e a telemetria) ----------------

    /**
     * true quando a mira está ligada, já recebeu a pose e o erro é pequeno.
     */
    public boolean isOnTarget() {
        return autoAimEnabled
                && poseReceived
                && Math.abs(error) <= ON_TARGET_TOLERANCE_DEG;
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

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Hive Alvo", aimingHiveA ? "A (Y > 72)" : "B (Y < 72)");
        telemetry.addData("Field Angle", fieldAngle);
        telemetry.addData("Turret Angle", currentAngle);
        telemetry.addData("Turret Target", targetAngle);
        telemetry.addData("Error", error);
        telemetry.addData("Target Rate (deg/s)", targetRate);
        telemetry.addData("Turret Speed (deg/s)", turretSpeed);
        telemetry.addData("FF Power", ffPower);
        telemetry.addData("On Target", isOnTarget());
        telemetry.addData("Servo Power", power);
        telemetry.addData("Limit Blocked", limitBlocked);
    }

    // ---------------- Utilitários ----------------

    // Normaliza ângulo para -180 a +180 graus
    private double wrapAngle(double angle) {

        while (angle > 180) {
            angle -= 360;
        }

        while (angle < -180) {
            angle += 360;
        }

        return angle;
    }

    // Limita um valor entre min e max
    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}