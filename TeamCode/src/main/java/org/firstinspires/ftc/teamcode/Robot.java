package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.TurretNext;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;


public class Robot implements NextRobot {

    public final TurretNext turret = new TurretNext();

    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(turret);
    }
}