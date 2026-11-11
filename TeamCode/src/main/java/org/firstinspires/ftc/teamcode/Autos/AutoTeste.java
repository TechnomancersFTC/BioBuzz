package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.storage.paths.startAuto;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanisms.FlowerTake;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.storage.paths;
import org.firstinspires.ftc.teamcode.Mechanisms.Torreta;

import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.OpModeHook;

@NextAutonomous (name = "AutoTeste")
public class AutoTeste extends NextOpMode {

    private final Robot robot;
    private Follower follower;
    private DcMotor fl, bl, br, fr;
    private Torreta torreta;

    Intake Intake = new Intake();
    FlowerTake flowerTake = new FlowerTake();

    public AutoTeste(Robot robot) {
        super(robot);

        this.robot = robot;

        follower = Constants.create(hardwareMap);
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);

        torreta = new Torreta(hardwareMap, true);

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
    }

    @Override
    public void periodic() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());

        if (follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
            telemetry.addData("Path number", follower.pathIndex());
        }

        torreta.update(follower.pose());

        if (torreta.isOnTarget()){}

        telemetry.update();

    }


    public Command autoRoutine() {
        return sequential(
                waitMs(2000),
                gardenIntake,
                firstFlower,
                secondFlower,
                follow(follower, paths.path4()),
                follow(follower, paths.path5())

        );
    }

    Command gardenIntake = parallel(
            follow(follower, paths.path1()),
            Intake.run()
    );

    Command firstFlower = sequential(
            follow(follower, paths.path2()),
            flowerTake.open(),
            follow(follower, paths.flowerTake()),
            Intake.run(),
            waitMs(1000),
            Intake.stop(),
            follow(follower, paths.flowerBack()),
            flowerTake.close()
    );

    Command secondFlower = sequential(
            follow(follower, paths.path3()),
            flowerTake.open(),
            follow(follower, paths.flowerTake2()),
            Intake.run(),
            waitMs(1000),
            Intake.stop(),
            follow(follower, paths.flowerBack2()),
            flowerTake.close()
    );






}