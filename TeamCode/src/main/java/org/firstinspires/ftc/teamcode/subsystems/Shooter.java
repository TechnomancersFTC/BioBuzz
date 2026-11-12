package org.firstinspires.ftc.teamcode.subsystems;
import static dev.nextftc.units.Units.DegreesPerSecond;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
public class Shooter implements Mechanism {

    private NextMotor shooter = new NextMotor("shooter");

    public void initialize() {

        shooter.getVelocityConstants()
                .withP(0.01)
                .withI(0.0)
                .withD(0.0);

        shooter.setVelocitySetpoint(
                DegreesPerSecond.of(18000)
        );
    }
}
