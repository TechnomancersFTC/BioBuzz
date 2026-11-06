package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Torreta;

/**
 * Teste: TeleOp field centric + mecanismo Torreta (mira automática nas Hives).
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
public class TorretaField extends OpMode {

    private Follower follower;
    private Torreta torreta;

    // Pose inicial do robô
    private static final double START_X = 87.4;
    private static final double START_Y = 133.8;
    private static final double START_HEADING_DEG = 270.0;

    // Direção do campo para a qual o driver está olhando (graus).
    // O robô começa em 270 (virado para -Y), então o driver está
    // olhando para -Y: empurrar o analógico para frente = andar em -Y.
    // Se o "frente" do campo estiver errado, ajuste aqui (ex.: 90).
    private static final double DRIVER_FACING_DEG = 270.0;

    // Variáveis do chassi (telemetria de diagnóstico)
    private double fieldX;
    private double fieldY;
    private double robotForward;
    private double robotLeft;
    double p1 = 0.8;
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

        // Zera o encoder com a torreta apontando para frente
        torreta = new Torreta(hardwareMap, true);

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
        torreta.update(robot);

        // ---------------- Telemetria ----------------

        telemetry.addData("Drive Field X", fieldX);
        telemetry.addData("Drive Field Y", fieldY);
        telemetry.addData("Drive Robot Fwd", robotForward);
        telemetry.addData("Drive Robot Left", robotLeft);

        telemetry.addData("Robot X", robot.x());
        telemetry.addData("Robot Y", robot.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robot.heading()));

        torreta.addTelemetry(telemetry);

        telemetry.update();
    }

    @Override
    public void stop() {
        torreta.stop();
    }
}