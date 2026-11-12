package org.firstinspires.ftc.teamcode.Mechanisms;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import com.pedropathing.ivy.Command;

public class Intake implements Mechanism {

    private final NextMotor intake = new NextMotor("intake");
    private final NextMotor intake2 = new NextMotor("intake2");

    public Command run() {
        return instant(() -> {
            intake.setThrottle(-1);
            intake2.setThrottle(-1);
        });
    }
    public Command reverse() {
        return instant(() -> {
            intake.setThrottle(1.0);
            intake2.setThrottle(1.0);
        });
    }

    public Command stop() {
        return instant(() -> {
            intake.setThrottle(0.0);
            intake2.setThrottle(0.0);
        });
    }
}