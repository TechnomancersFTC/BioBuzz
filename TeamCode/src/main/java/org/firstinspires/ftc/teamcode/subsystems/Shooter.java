package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Shooter implements Mechanism {

    private NextMotor shooter = new NextMotor("shooter");

}
