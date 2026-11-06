package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.sensors.NextDistanceSensor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    NextMotor motor = new NextMotor("intakeMotor");
    NextDistanceSensor sensor = new NextDistanceSensor("intakeSensor");

    @Override
    public void periodic() {
        sensor.update();
        if (sensor.isWithinDistance(2.0)) {
            motor.setThrottle(0.0);
        }
    }

    public Command run() {
        return instant(() -> motor.setThrottle(1.0));
    }
}