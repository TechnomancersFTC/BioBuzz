
package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.storage.paths;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.drive.DriveCommands;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
@Config
@NextTeleop(name = "TeleOp Turret Next")
public class TeleOpTurretNext extends NextOpMode {

    private static final String FRONT_LEFT = "fl";
    private static final String FRONT_RIGHT = "fr";
    private static final String BACK_LEFT = "bl";
    private static final String BACK_RIGHT = "br";

    private static final Pose START_POSE = paths.startPoseteleop;
    private static final boolean RESET_TURRET_ENCODER = true;

    private static final double GOAL_X = 70.75;
    private static final double GOAL_Y = 84;
    public static volatile double reverse = 0.2;

    private final Robot robot;
    private final Follower follower;

    private final NextMotor frontLeft = new NextMotor(FRONT_LEFT);
    private final NextMotor frontRight = new NextMotor(FRONT_RIGHT);
    private final NextMotor backLeft = new NextMotor(BACK_LEFT);
    private final NextMotor backRight = new NextMotor(BACK_RIGHT);

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

    private double goalDistance() {
        Pose p = follower.pose();
        return Math.hypot(GOAL_X - p.x(), GOAL_Y - p.y());
    }

    private Command lockAndIntakeSequence() {
        return sequential(

                robot.intake.reverse(),
                waitMs(reverse),
                robot.intake.stop(),
                robot.lock.open(),
                robot.intake.run(),
                waitMs(1000),
                robot.intake.stop(),
                robot.lock.close()
        );
    }

    @Override
    public void start() {

        robot.shooter.shootAuto(this::goalDistance);

        CommandGamepad gp1 = new CommandGamepad(gamepad1);

        gp1.leftBumper().whileTrue(robot.intake.run());
        gp1.leftBumper().onFalse(robot.intake.stop());

        gp1.a().whileTrue(robot.intake.reverse());
        gp1.a().onFalse(robot.intake.stop());
        gp1.b().onTrue(robot.turret.toggleShootWhileMoving());
        gp1.rightBumper().onTrue(lockAndIntakeSequence());
        DriveCommands.mecanumDriveFieldCentric(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                gamepad1,
                () -> Math.toRadians(
                        Math.toDegrees(follower.pose().heading()) + 180
                )
        ).schedule();
    }

    @Override
    public void periodic() {
        follower.update();

        Pose pose = follower.pose();

        Telemetry.log("X", pose.x());
        Telemetry.log("Y", pose.y());
        Telemetry.log(
                "Heading (deg)",
                Math.toDegrees(pose.heading())
        );
        Telemetry.log("Goal Dist", goalDistance());

        Telemetry.log(
                "Auto Aim",
                robot.turret.isAutoAimEnabled()
        );

        robot.turret.logTelemetry();
    }

    @Override
    public void end() {
        robot.turret.stop();
        robot.shooter.stop();
        robot.intake.stop().schedule();
        robot.lock.close().schedule();
    }
}
