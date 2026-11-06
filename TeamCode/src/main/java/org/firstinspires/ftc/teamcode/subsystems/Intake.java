package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.groups.Groups;

import static com.pedropathing.ivy.groups.Groups.parallel;

import java.security.acl.Group;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.opmode.NextUtility;
import dev.nextftc.units.measuretypes.Voltage;

public class Intake implements Mechanism {

    private NextMotor intake = new NextMotor("intake");
    private NextMotor intake2 = new NextMotor("intake2");




}
