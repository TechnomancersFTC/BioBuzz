
package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

@Config
public class Lock implements Mechanism {
    public static double OPEN_POSITION = 0.8;
    public static double CLOSED_POSITION = 1.0;

    private final NextServo servo = new NextServo("sLock");

    public Command setPosition(double position) {
        return instant(() -> {
            servo.setPosition(clip(position));
        });
    }

    public Command open() {
        return setPosition(OPEN_POSITION);
    }

    public Command close() {
        return setPosition(CLOSED_POSITION);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    private double clip(double position) {
        return Math.max(0.0, Math.min(1.0, position));
    }
}
