package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.storage.paths;
import org.firstinspires.ftc.teamcode.Mechanisms.Torreta;

@Autonomous
public class AutoTeste extends OpMode {

    private Follower follower;
    private DcMotor fl, bl, br, fr, shooter, intake, intake2;
    private Servo servoE, servoD;
    private Torreta torreta;
    double p1 = 0.85;
    double p0 = 0;
    double p3 = 0.4;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        servoE = hardwareMap.get(Servo.class, "f1");
        servoD = hardwareMap.get(Servo.class, "f2");

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
        servoD.setDirection(Servo.Direction.REVERSE);

        torreta = new Torreta(hardwareMap, true);

        Scheduler.reset();
        follower.setPose(paths.startAuto);
        follower.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
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

        shooter.setPower(0.7);
    }

    public Command ligarIntake() {
        return instant(() -> {
            intake.setPower(1.0);
            intake2.setPower(1.0);
        });
    }

    public Command desligarIntake() {
        return instant(() -> {
            intake.setPower(0.0);
            intake2.setPower(0.0);
        });
    }

    public Command flowerIntake(){
        return instant(() -> {
           servoE.setPosition(p1);
           servoD.setPosition(p1);
        });
    }

    public Command flowerShoot(){
        return instant(() -> {
            servoE.setPosition(p3);
            servoD.setPosition(p3);
        });
    }
    public Command flowerCima(){
        return instant(() -> {
            servoE.setPosition(p0);
            servoD.setPosition(p0);
        });
    }

    public Command autoRoutine() {
        return sequential(

                waitMs(2000),
                ligarIntake(),
                waitMs(700),
                follow(follower, paths.path1()),
                desligarIntake(),
//                flowerIntake(),
                follow(follower, paths.path2()),
                ligarIntake(),
                waitMs(2000),
                desligarIntake(),
                follow(follower, paths.path3()),
//                flowerShoot(),
                follow(follower, paths.path4()),
                ligarIntake(),
                waitMs(2000),
                follow(follower, paths.path5())

        );
    }




}