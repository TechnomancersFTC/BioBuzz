package org.firstinspires.ftc.teamcode.storage;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class poses{

    public static double HiveAltura  = 169;
    static PoseFactory Factory = PoseFactory.degrees();
    //poses teste
    public static Pose upperBlueHive = Factory.of(83.8, 87.4,90);
    public static Pose lowerBlueHive = Factory.of(83.8, 56.6,270);
    Pose upperRedHive = Factory.of(58, 83.5, 90);
    Pose lowerRedHive = Factory.of(59, 59, 270);

}
