package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.storage.paths;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.drive.DriveCommands;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

// Importe com Alt+Enter: DriveCommands (do NextFTC).
// Se NextMotor não resolver em dev.nextftc.hardware.actuators, use Alt+Enter também.

/**
 * TeleOp: direção field-centric do NextFTC (DriveCommands) + TurretNext.
 *
 * O Pedro aqui serve só para a ODOMETRIA (pose): a torreta usa a pose para
 * mirar e o field-centric usa o heading dela. Quem move as rodas é o NextFTC,
 * por isso NÃO chamamos startTeleOpDrive() nem setTeleOpDrive().
 *
 * Controles:
 *   Stick esquerdo  = andar (relativo ao campo)
 *   Stick direito X = girar
 *   Gamepad1 right bumper = liga/desliga a mira da torreta
 */
@NextTeleop(name = "TeleOp Turret Next")
public class TeleOpTurretNext extends NextOpMode {

    // Nomes dos motores no hardwareMap (os mesmos do seu autônomo)
    private static final String FRONT_LEFT = "fl";
    private static final String FRONT_RIGHT = "fr";
    private static final String BACK_LEFT = "bl";
    private static final String BACK_RIGHT = "br";

    // Pose inicial do robô no campo. Por padrão usa a mesma do autônomo (paths.start).
    // Se este TeleOp roda DEPOIS do autônomo, troque pela pose final dele
    // (ex.: guardar follower.pose() numa variável estática no fim do auto).
    private static final Pose START_POSE = paths.start;

    // true  = zera o encoder da torreta no INIT (torreta apontando pra frente do robô)
    // false = mantém a contagem (TeleOp depois do autônomo)
    private static final boolean RESET_TURRET_ENCODER = true;

    private final Robot robot;
    private final Follower follower;

    private final NextMotor frontLeft = new NextMotor(FRONT_LEFT);
    private final NextMotor frontRight = new NextMotor(FRONT_RIGHT);
    private final NextMotor backLeft = new NextMotor(BACK_LEFT);
    private final NextMotor backRight = new NextMotor(BACK_RIGHT);

    private boolean lastBumper = false;

    public TeleOpTurretNext(Robot robot) {
        super(robot);
        this.robot = robot;

        // Direção dos motores igual à do seu autônomo (fr e br invertidos)
        frontRight.setDirection(NextMotor.Direction.REVERSE);
        backRight.setDirection(NextMotor.Direction.REVERSE);

        // Pedro (só odometria), criado como no autônomo
        follower = Constants.create(hardwareMap);
        follower.setPose(START_POSE);
        follower.update();

        robot.turret.init(hardwareMap, RESET_TURRET_ENCODER, follower::pose);
    }

    @Override
    public void start() {
        // Direção field-centric do NextFTC. O heading vem da pose do Pedro (radianos).
        // O comando fica rodando todo loop lendo o gamepad.
        DriveCommands.mecanumDriveFieldCentric(
                frontLeft, frontRight, backLeft, backRight,
                gamepad1,
                () -> follower.pose().heading()
        ).schedule();
    }

    @Override
    public void periodic() {
        // Atualiza a odometria (a torreta e o field-centric leem a pose)
        follower.update();

        // Liga/desliga a mira na borda de subida do botão
        boolean bumper = gamepad1.right_bumper;
        if (bumper && !lastBumper) {
            robot.turret.setAutoAimEnabled(!robot.turret.isAutoAimEnabled());
        }
        lastBumper = bumper;

        Pose pose = follower.pose();
        Telemetry.log("X", pose.x());
        Telemetry.log("Y", pose.y());
        Telemetry.log("Heading (deg)", Math.toDegrees(pose.heading()));
        Telemetry.log("Auto Aim", robot.turret.isAutoAimEnabled());
        robot.turret.logTelemetry();
    }

    @Override
    public void end() {
        robot.turret.stop();
    }
}