package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextServo;

import dev.nextftc.robot.Mechanism;


public class FlowerTake implements Mechanism {

    private final NextServo servo1 = new NextServo("f2");
    private final NextServo servo2 = new NextServo("f1");

    public Command baixo() {
        return instant(() -> {
            servo1.setPosition(0.2);
            servo2.setPosition(0.8);
        });
    }
    public Command shoot() {
        return instant(() -> {
            servo1.setPosition(.4);
            servo2.setPosition(.4);
        });
    }
    public Command alto() {
        return instant(() -> {
            servo1.setPosition(0.8);
            servo2.setPosition(0.2);
        });
    }
}