
package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.storage.paths;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.drive.DriveCommands;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "TeleOp Turret Next")
public class TeleOpTurretNext extends NextOpMode {

    private static final String FRONT_LEFT = "fl";
    private static final String FRONT_RIGHT = "fr";
    private static final String BACK_LEFT = "bl";
    private static final String BACK_RIGHT = "br";
    private static final Pose START_POSE = paths.startPoseteleop;
    private static final boolean RESET_TURRET_ENCODER = true;

    private final Robot robot;
    private final Follower follower;
    private Servo servoE, servoD;

    double p1 = 0.85;
    double p0 = 0;

    private final NextMotor frontLeft = new NextMotor(FRONT_LEFT);
    private final NextMotor frontRight = new NextMotor(FRONT_RIGHT);
    private final NextMotor backLeft = new NextMotor(BACK_LEFT);
    private final NextMotor backRight = new NextMotor(BACK_RIGHT);

    private boolean lastBumper = false;
    boolean fState = false;

    public TeleOpTurretNext(Robot robot) {
        super(robot);
        this.robot = robot;

        frontRight.setDirection(NextMotor.Direction.REVERSE);
        backRight.setDirection(NextMotor.Direction.REVERSE);

        follower = Constants.create(hardwareMap);
        follower.setPose(START_POSE);
        follower.update();

        robot.turret.init(
                hardwareMap,
                RESET_TURRET_ENCODER,
                follower::pose
        );
    }

    @Override
    public void start() {

        CommandGamepad gp1 = new CommandGamepad(gamepad1);

        gp1.leftBumper().whileTrue(robot.intake.run());
        gp1.leftBumper().onFalse(robot.intake.stop());

        gp1.a().whileTrue(robot.intake.reverse());
        gp1.a().onFalse(robot.intake.stop());

        DriveCommands.mecanumDriveFieldCentric(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                gamepad1,
                () -> follower.pose().heading()
        ).schedule();
        servoE = hardwareMap.get(Servo.class, "f1");
        servoD = hardwareMap.get(Servo.class, "f2");
    }

    @Override
    public void periodic() {
        follower.update();

        boolean aPressed = gamepad1.a;

        boolean bumper = gamepad1.right_bumper;

        if (bumper && !lastBumper) {
            robot.turret.setAutoAimEnabled(
                    !robot.turret.isAutoAimEnabled()
            );
        }


        lastBumper = bumper;


        Pose pose = follower.pose();

        Telemetry.log("X", pose.x());
        Telemetry.log("Y", pose.y());
        Telemetry.log(
                "Heading (deg)",
                Math.toDegrees(pose.heading())
        );

        Telemetry.log(
                "Auto Aim",
                robot.turret.isAutoAimEnabled()
        );

        robot.turret.logTelemetry();

        if (gamepad1.xWasPressed()) {
            fState = !fState;
            if (fState) {
                servoD.setPosition(p0); // Posição ON // direita
                servoE.setPosition(p1);//esquerda
            } else {
                servoD.setPosition(p1); // Posição OFF
                servoE.setPosition(p0);

            }
        }
    }

    @Override
    public void end() {
        robot.turret.stop();
        robot.intake.stop().schedule();
    }
}
