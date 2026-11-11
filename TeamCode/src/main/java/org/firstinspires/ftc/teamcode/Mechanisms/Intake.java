package org.firstinspires.ftc.teamcode.Mechanisms;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.waitMs;

import android.provider.ContactsContract;

public class Intake implements Mechanism {

    private final NextMotor intake = new NextMotor("intake");
    private final NextMotor intake2 = new NextMotor("intake2");
    private final NextServo lock = new NextServo("sLock");

    double p1 = 0.8;
    double p0 = 0;

    public Command run() {
        return instant(() -> {
            lock.setPosition(p1);
            intake.setThrottle(-1.0);
            intake2.setThrottle(-1.0);
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