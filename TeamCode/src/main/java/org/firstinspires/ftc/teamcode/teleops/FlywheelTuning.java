
package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.storage.paths;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;

@Config
@NextTeleop(name = "Flywheel Tuning")
public class FlywheelTuning extends NextOpMode {

    private final Robot robot;
    private Follower follower;

    private static final double HIVE_A_X = 70.75;
    private static final double HIVE_A_Y = 84.0;

    private static final double HIVE_B_X = 70.75;
    private static final double HIVE_B_Y = 84.0;

    private static final double FIELD_MIDDLE_Y = 72.0;

    private double distanceToGoal;

    private boolean lastA = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    public FlywheelTuning(Robot robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void start() {

        follower = Constants.create(hardwareMap);

        follower.setPose(paths.startPoseteleop);
        follower.update();

        robot.turret.init(
                hardwareMap,
                true,
                follower::pose
        );

        robot.shooter.stop();
        robot.intake.stop().schedule();
    }

    private Command lockAndIntakeSequence() {
        return sequential(
                robot.lock.open(),
                robot.intake.run(),
                waitMs(1000),
                robot.intake.stop(),
                robot.lock.close()
        );
    }

    @Override
    public void periodic() {

        follower.update();

        // A: liga/desliga shooter
        boolean aPressed = gamepad1.a;

        if (aPressed && !lastA) {

            if (robot.shooter.isRunning()) {
                robot.shooter.stop();
            } else {
                robot.shooter.shoot();
            }
        }

        lastA = aPressed;

        // LEFT BUMPER: intake enquanto segura
        boolean leftBumperPressed = gamepad1.left_bumper;

        if (leftBumperPressed && !lastLeftBumper) {
            robot.intake.run().schedule();
        }

        if (!leftBumperPressed && lastLeftBumper) {
            robot.intake.stop().schedule();
        }

        lastLeftBumper = leftBumperPressed;

        // RIGHT BUMPER: sequência lock + intake
        boolean rightBumperPressed = gamepad1.right_bumper;

        if (rightBumperPressed && !lastRightBumper) {
            lockAndIntakeSequence().schedule();
        }

        lastRightBumper = rightBumperPressed;

        Pose pose = follower.pose();

        double targetX;
        double targetY;

        if (pose.y() > FIELD_MIDDLE_Y) {
            targetX = HIVE_A_X;
            targetY = HIVE_A_Y;
        } else {
            targetX = HIVE_B_X;
            targetY = HIVE_B_Y;
        }

        distanceToGoal = Math.hypot(
                targetX - pose.x(),
                targetY - pose.y()
        );

        Telemetry.log("Robot X", pose.x());
        Telemetry.log("Robot Y", pose.y());

        Telemetry.log(
                "Robot Heading",
                Math.toDegrees(pose.heading())
        );

        Telemetry.log("Distance to Goal (in)", distanceToGoal);

        Telemetry.log(
                "Flywheel Running",
                robot.shooter.isRunning()
        );

        Telemetry.log(
                "Target Velocity",
                robot.shooter.getTarget()
        );

        Telemetry.log(
                "Current Velocity",
                robot.shooter.getVelocity()
        );

        Telemetry.log(
                "Velocity Error",
                robot.shooter.getVelocityError()
        );

        Telemetry.log(
                "Motor Power",
                robot.shooter.getPower()
        );

        robot.turret.logTelemetry();
    }

    @Override
    public void end() {

        robot.shooter.stop();

        robot.intake.stop().schedule();

        robot.turret.stop();

        robot.lock.close().schedule();
    }
}
