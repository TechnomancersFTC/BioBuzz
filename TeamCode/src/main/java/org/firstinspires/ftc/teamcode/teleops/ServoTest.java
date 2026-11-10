
package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@Config
@NextTeleop(name = "Servo Test")
public class ServoTest extends NextOpMode {

    private final Robot robot;

    public static double POSITION_A = 0.0;
    public static double POSITION_B = 0.25;
    public static double POSITION_X = 0.5;
    public static double POSITION_Y = 0.75;
    public static double POSITION_RB = 1.0;

    private boolean lastA;
    private boolean lastB;
    private boolean lastX;
    private boolean lastY;
    private boolean lastRB;

    private double targetPosition = 0.5;

    public ServoTest(Robot robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void periodic() {

        if (gamepad1.a && !lastA) {
            targetPosition = POSITION_A;
            robot.lock.setPosition(targetPosition).schedule();
        }

        if (gamepad1.b && !lastB) {
            targetPosition = POSITION_B;
            robot.lock.setPosition(targetPosition).schedule();
        }

        if (gamepad1.x && !lastX) {
            targetPosition = POSITION_X;
            robot.lock.setPosition(targetPosition).schedule();
        }

        if (gamepad1.y && !lastY) {
            targetPosition = POSITION_Y;
            robot.lock.setPosition(targetPosition).schedule();
        }

        if (gamepad1.right_bumper && !lastRB) {
            targetPosition = POSITION_RB;
            robot.lock.setPosition(targetPosition).schedule();
        }

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastRB = gamepad1.right_bumper;

        Telemetry.log("Servo Target", targetPosition);
    }
}
