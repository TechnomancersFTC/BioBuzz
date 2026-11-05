package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Teste: TeleOp field centric + torreta com mira automática na Hive azul.
 *
 * O chassi usa o Follower do Pedro Pathing (motores já configurados
 * em Constants), mas só para aplicar potência em modo robot centric.
 * A rotação field centric é feita aqui, com o heading da odometria.
 *
 * Controles (gamepad1):
 *  - Analógico esquerdo: translação em relação ao campo
 *  - Analógico direito (X): rotação
 */
@TeleOp(name = "Teste Torreta + Field Centric")
public class Help extends OpMode {

    // Hardware da torreta
    private CRServo turretServo;
    private DcMotorEx encoder, intake, intake2, shooter;
    private IMU imu;
    private Servo f2, fesquerda;

    // Pedro Pathing 3
    private Follower follower;

    // Pose inicial do robô
    private static final double START_X = 87.4;
    private static final double START_Y = 133.8;

    private static final double START_HEADING_DEG = 270.0;

    // Direção do campo para a qual o driver está olhando (graus).
    // O robô começa em 270 (virado para -Y), então o driver está
    // olhando para -Y: empurrar o analógico para frente = andar em -Y.
    // Se o "frente" do campo estiver errado, ajuste aqui (ex.: 90).
    private static final double DRIVER_FACING_DEG = 270.0;

    // Coordenadas da Hive azul
    private static final double HIVE_X = 84.5;
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
    private static final double SAFETY_MARGIN_DEG = 3.0;

    // Controle proporcional
    private static final double KP = 0.04;
    private static final double MAX_POWER = 0.35;
    private static final double DEADBAND = 1.0;

    // Variáveis de controle
    private double currentAngle;
    private double fieldAngle;
    private double targetAngle;
    private double error;
    private double power;
    private boolean limitBlocked;

    // Variáveis do chassi (telemetria de diagnóstico)
    private double fieldX;
    private double fieldY;
    private double robotForward;
    private double robotLeft;

    boolean fState = false;
    double p1 = 0.85;
    double p0 = 0;

    @Override
    public void init() {

        // Inicializa Pedro Pathing
        follower = Constants.create(hardwareMap);

        follower.setPose(new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING_DEG)
        ));

        // Inicializa servo
        turretServo = hardwareMap.get(
                CRServo.class, "storreta2"
        );

        // Inicializa encoder
        encoder = hardwareMap.get(
                DcMotorEx.class, "encoder"
        );

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        imu = hardwareMap.get(IMU.class, "imu");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        f2 = hardwareMap.get(Servo.class, "f2");
        fesquerda = hardwareMap.get(Servo.class, "fesquerda");

        // Zera encoder com a torreta apontando para frente
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.resetYaw();
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        f2.setDirection(Servo.Direction.REVERSE);


        // =========================
        // VARIÁVEIS DOS BOTÕES
        // =========================


        turretServo.setPower(0);

        telemetry.addLine("Torreta + Field Centric pronto");
        telemetry.update();
    }

    @Override
    public void start() {
        // Passa o controle do chassi para o gamepad
        follower.manual();

    }

    @Override
    public void loop() {

        // ---------------- Chassi field centric ----------------

        // Entradas do driver: frente (+), esquerda (+), giro anti-horário (+)
        double forward = -gamepad1.left_stick_y;
        double left = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // 1) Referencial do driver -> referencial do campo
        double facing = Math.toRadians(DRIVER_FACING_DEG);
        fieldX = forward * Math.cos(facing) - left * Math.sin(facing);
        fieldY = forward * Math.sin(facing) + left * Math.cos(facing);

        // 2) Referencial do campo -> referencial do robô (gira por -heading,
        //    usando o mesmo heading da odometria que a mira usa)
        double heading = follower.pose().heading();
        robotForward = fieldX * Math.cos(heading) + fieldY * Math.sin(heading);
        robotLeft = -fieldX * Math.sin(heading) + fieldY * Math.cos(heading);

        // true = robot centric: a rotação field centric já foi feita acima,
        // então o Pedro só aplica as potências no chassi
        follower.manual(robotForward, robotLeft, turn);

        // Atualiza odometria e aplica a potência do chassi
        follower.update();

        // ---------------- Torreta ----------------

        Pose robot = follower.pose();

        double robotX = robot.x();
        double robotY = robot.y();
        double robotHeading = Math.toDegrees(robot.heading());

        // Ângulo atual da torreta (0 = frente do robô, sem wrap)
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

        // Ângulo da Hive relativo à frente do robô, dentro da faixa mecânica
        targetAngle = clip(
                wrapAngle(fieldAngle - robotHeading),
                TURRET_MIN_DEG + SAFETY_MARGIN_DEG,
                TURRET_MAX_DEG - SAFETY_MARGIN_DEG
        );

        // Erro sem wrap: a torreta nunca cruza +-180
        error = targetAngle - currentAngle;

        if (Math.abs(error) <= DEADBAND) {
            power = 0;
        } else {
            power = clip(KP * error, -MAX_POWER, MAX_POWER);
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

        turretServo.setPower(SERVO_SIGN * power);


        // -------- Botões ---------

        if (gamepad1.right_bumper) {
            intake.setPower(1);
            intake2.setPower(1);

        } else if (gamepad1.a) {
            intake.setPower(-0.5);
            intake2.setPower(-0.5);
        } else {
            intake.setPower(0);
            intake2.setPower(0);
        }

        //shooter.setPower(0.67);

        // Alternar servos f1 e f2 (ON/OFF com o botão X)
        if (gamepad1.xWasPressed()) {
            fState = !fState;
            if (fState) {
                f2.setPosition(p1); // Posição ON // direita
                fesquerda.setPosition(p1);//esquerda
            } else {
                f2.setPosition(p0); // Posição OFF
                fesquerda.setPosition(p0);

            }
        }

        // ---------------- Telemetria ----------------

        telemetry.addData("Drive Field X", fieldX);
        telemetry.addData("Drive Field Y", fieldY);
        telemetry.addData("Drive Robot Fwd", robotForward);
        telemetry.addData("Drive Robot Left", robotLeft);

        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Robot Heading", robotHeading);

        telemetry.addData("Field Angle", fieldAngle);
        telemetry.addData("Turret Angle", currentAngle);
        telemetry.addData("Turret Target", targetAngle);
        telemetry.addData("Error", error);
        telemetry.addData("Servo Power", power);
        telemetry.addData("Limit Blocked", limitBlocked);

        telemetry.addData("Posição servos: ", fState);


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