package org.firstinspires.ftc.teamcode.Autos;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.storage.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextAutonomous;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.storage.paths.path4;
import static org.firstinspires.ftc.teamcode.storage.paths.path5;
import static org.firstinspires.ftc.teamcode.storage.paths.startAuto;

//COMMIT TESTE

@NextAutonomous(name = "BIO48", group = "BioBuzz")
public class Bio48 extends NextOpMode {

    private final Robot robot;
    private final Follower follower;


    private final NextServo lock = new NextServo("sLock");

    public Bio48(Robot robot) {
        super(robot);

        this.robot = robot;

        follower = Constants.create(hardwareMap);

        follower.setPose(startAuto);
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
        robot.turret.setAutoAimEnabled(true);
        robot.shooter.shootAuto(
                robot.turret::getAimDistance,
                robot.turret::getRadialSpeed
        );
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

    private Command autoRoutine() {
        Command gardenIntake = parallel(
                fechaLock(),
                robot.intake.run(),
                waitMs(2000),
                follow(follower, paths.path1())
        );

        Command firstFlower = sequential(
                follow(follower, paths.path2()),
                robot.flowerTake.open(),
                follow(follower, paths.flowerTake()),
                robot.intake.run(),
                waitMs(3000),
                robot.intake.stop(),
                follow(follower, paths.flowerBack()),
                robot.flowerTake.close()
        );

        Command secondFlower = sequential(
                follow(follower, paths.path3()),
                robot.flowerTake.open(),
                follow(follower, paths.flowerTake2()),
                robot.intake.run(),
                waitMs(1000),
                robot.intake.stop(),
                follow(follower, paths.flowerBack2()),
                robot.flowerTake.close()
        );

        return sequential(
                waitMs(2500),
                launch(),
                gardenIntake,
                firstFlower,
                secondFlower,
                follow(follower, path4()),
                waitMs(2000),
                follow(follower, path5())

      );
    }

    private Command launch(){
        return sequential(
                abreLock(),
                robot.intake.run(),
                waitMs(850),
                robot.intake.stop(),
                fechaLock()
        );
    }

    public Command abreLock() {
        return instant(() -> {
            lock.setPosition(0);
        });
    }

    public Command fechaLock() {
        return instant(() -> {
            lock.setPosition(0.8);
        });
    }

    @Override
    public void end() {
        robot.shooter.stop();
        robot.intake.stop().schedule();
        follower.stop();
    }
}
