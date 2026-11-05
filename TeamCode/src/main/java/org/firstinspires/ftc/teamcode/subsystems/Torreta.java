package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Teste Mira Hive Azul")
public class Torreta extends OpMode {

    // Hardware
    private CRServo turretServo;
    private DcMotorEx encoder;

    // Pedro Pathing 3
    private Follower follower;

    // Coordenadas da Hive azul
    private static final double HIVE_X = 87.5;
    private static final double HIVE_Y = 80.22;

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
    private static final double KP = 0.012;
    private static final double MAX_POWER = 0.35;
    private static final double DEADBAND = 1.0;

    // Variáveis de controle
    private double currentAngle;
    private double fieldAngle;
    private double turretFieldAngle;
    private double targetAngle;
    private double error;
    private double power;
    private boolean limitBlocked;

    @Override
    public void init() {

        // Inicializa Pedro Pathing
        follower = Constants.create(hardwareMap);

        follower.setPose(new Pose(
                87.4,
                133.8,
                Math.toRadians(270)
        ));

        // Inicializa servo
        turretServo = hardwareMap.get(
                CRServo.class, "storreta2"
        );

        // Inicializa encoder
        encoder = hardwareMap.get(
                DcMotorEx.class, "encoder"
        );

        // Zera encoder com a torreta apontando para frente
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretServo.setPower(0);

        telemetry.addLine("Mira automatica - Hive Azul");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Atualiza localização pela odometria
        follower.update();

        Pose robot = follower.pose();

        double robotX = robot.x();
        double robotY = robot.y();
        double robotHeading = Math.toDegrees(robot.heading());

        // Lê o ângulo atual da torreta (0 = frente do robô, sem wrap)
        currentAngle =
                ENCODER_SIGN
                        * encoder.getCurrentPosition()
                        * DEG_PER_TICK;

        // Ângulo absoluto da Hive em relação ao campo
        fieldAngle = Math.toDegrees(
                Math.atan2(
                        HIVE_Y - robotY,
                        HIVE_X - robotX
                )
        );

        // Orientação absoluta atual da torreta (apenas telemetria)
        turretFieldAngle = robotHeading + currentAngle;

        // Ângulo da Hive relativo à frente do robô (-180 a +180)
        targetAngle = wrapAngle(fieldAngle - robotHeading);

        // Mantém o alvo dentro da faixa mecânica
        targetAngle = clip(
                targetAngle,
                TURRET_MIN_DEG + SAFETY_MARGIN_DEG,
                TURRET_MAX_DEG - SAFETY_MARGIN_DEG
        );

        // Erro NO REFERENCIAL DA TORRETA e SEM wrap:
        // se o alvo estiver do outro lado do limite, a torreta
        // desenrola pelo caminho longo em vez de cruzar +-180.
        error = targetAngle - currentAngle;

        // Controle proporcional
        if (Math.abs(error) <= DEADBAND) {
            power = 0;
        } else {
            power = clip(
                    KP * error,
                    -MAX_POWER,
                    MAX_POWER
            );
        }

        // Trava de segurança: nunca empurra a torreta para fora dos limites
        limitBlocked = false;

        if (currentAngle >= TURRET_MAX_DEG && power > 0) {
            power = 0;
            limitBlocked = true;
        }

        if (currentAngle <= TURRET_MIN_DEG && power < 0) {
            power = 0;
            limitBlocked = true;
        }

        // Aciona servo
        turretServo.setPower(SERVO_SIGN * power);

        // Telemetria
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Robot Heading", robotHeading);

        telemetry.addData("Field Angle", fieldAngle);
        telemetry.addData("Turret Angle", currentAngle);
        telemetry.addData("Turret Target", targetAngle);
        telemetry.addData(
                "Turret Field Angle",
                wrapAngle(turretFieldAngle)
        );

        telemetry.addData("Error", error);
        telemetry.addData("Servo Power", power);
        telemetry.addData("Limit Blocked", limitBlocked);


        telemetry.update();
    }

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

    @Override
    public void stop() {
        turretServo.setPower(0);
    }
}