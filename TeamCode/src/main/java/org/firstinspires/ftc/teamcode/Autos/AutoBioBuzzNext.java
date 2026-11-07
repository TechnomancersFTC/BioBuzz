
package org.firstinspires.ftc.teamcode.Autos;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.constants.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextAutonomous;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@NextAutonomous(name = "Auto BioBuzz Next")
public class AutoBioBuzzNext extends NextOpMode {

    private final Robot robot;
    private final Follower follower;

    public AutoBioBuzzNext(Robot robot) {
        super(robot);

        this.robot = robot;

        follower = Constants.create(hardwareMap);

        follower.setPose(paths.start);
        follower.update();

        robot.turret.init(
                hardwareMap,
                true,
                follower::pose
        );
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void periodic() {

        follower.update();

        Pose pose = follower.pose();

        Telemetry.log("X", pose.x());
        Telemetry.log("Y", pose.y());
        Telemetry.log(
                "Heading",
                Math.toDegrees(pose.heading())
        );

        Telemetry.log("Follower Busy", follower.isBusy());
        Telemetry.log("Flywheel Running", robot.shooter.isRunning());
        Telemetry.log("Flywheel Target", robot.shooter.getTarget());
        Telemetry.log("Flywheel Velocity", robot.shooter.getVelocity());

        robot.turret.logTelemetry();
    }

    private com.pedropathing.ivy.Command autoRoutine() {

        return sequential(

                waitMs(2000),

                instant(() -> robot.shooter.shoot()),

                instant(() -> robot.intake.run().schedule()),

                waitMs(700),

                instant(() -> robot.intake.stop().schedule()),

                follow(follower, paths.path1()),

                follow(follower, paths.path2()),

                instant(() -> robot.intake.run().schedule()),

                waitMs(2000),

                instant(() -> robot.intake.stop().schedule()),

                follow(follower, paths.path3()),
                follow(follower, paths.path4()),

                instant(() -> robot.intake.run().schedule()),

                waitMs(2000),
                instant(() -> robot.intake.stop().schedule()),
                follow(follower, paths.path5())

        );
    }

    @Override
    public void end() {

        robot.shooter.stop();
        robot.intake.stop().schedule();

        follower.stop();
    }
}
