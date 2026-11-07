
package org.firstinspires.ftc.teamcode.teleops;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop(name = "FlywheelTuning")
public class FlywheelTuning extends NextOpMode {

    private final Robot robot;

    private boolean lastA = false;
    private boolean lastLeftBumper = false;

    public FlywheelTuning(Robot robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void start() {
        robot.shooter.stop();
        robot.intake.stop().schedule();
    }

    @Override
    public void periodic() {

        boolean buttonA = gamepad1.a;

        if (buttonA && !lastA) {
            if (robot.shooter.isRunning()) {
                robot.shooter.stop();
            } else {
                robot.shooter.shoot();
            }
        }

        lastA = buttonA;

        boolean leftBumper = gamepad1.left_bumper;

        if (leftBumper && !lastLeftBumper) {
            robot.intake.run().schedule();
        }

        if (!leftBumper && lastLeftBumper) {
            robot.intake.stop().schedule();
        }

        lastLeftBumper = leftBumper;

        Telemetry.log("Flywheel Running", robot.shooter.isRunning());
        Telemetry.log("Target Velocity", robot.shooter.getTarget());
        Telemetry.log("Current Velocity", robot.shooter.getVelocity());
        Telemetry.log("Velocity Error", robot.shooter.getVelocityError());
        Telemetry.log("Motor Power", robot.shooter.getPower());
    }

    @Override
    public void end() {
        robot.shooter.stop();
        robot.intake.stop().schedule();
    }
}
