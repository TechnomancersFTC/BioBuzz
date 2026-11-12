
package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@Config
@NextTeleop(name = "Teste Lock Dashboard")
public class LockTest extends NextOpMode {

    private final Robot robot;

    public LockTest(Robot robot) {
        super(robot);
        this.robot = robot;
    }


    @Override
    public void periodic() {
        robot.lock.updateDashboardPosition();

        Telemetry.log(
                "Lock Position",
                robot.lock.getPosition()
        );

        Telemetry.log(
                "Dashboard Position",
                RobotLockPosition()
        );
    }

    private double RobotLockPosition() {
        return org.firstinspires.ftc.teamcode.Mechanisms.Lock.DASHBOARD_POSITION;
    }
}
