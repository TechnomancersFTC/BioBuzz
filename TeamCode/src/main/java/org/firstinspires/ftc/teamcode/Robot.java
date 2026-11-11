package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Mechanisms.FlowerTake;
import org.firstinspires.ftc.teamcode.Mechanisms.Lock;
import org.firstinspires.ftc.teamcode.Mechanisms.TurretNext;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class Robot implements NextRobot {

    public final TurretNext turret = new TurretNext();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final FlowerTake flowerTake = new FlowerTake();
    public final Lock lock = new Lock();

    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(
                turret,
                intake,
                shooter,
                flowerTake,
                lock
        );
    }
}