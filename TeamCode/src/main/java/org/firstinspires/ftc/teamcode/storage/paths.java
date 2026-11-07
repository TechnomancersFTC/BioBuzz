package org.firstinspires.ftc.teamcode.storage;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class paths{


    //Pontos
    private static final PoseFactory poseFactory = PoseFactory.degrees();
    public static final Pose startPoseteleop = poseFactory.of(9.4, 7.8, 90);
    public static final Pose startAuto = poseFactory.of(87.4, 133.8, 270);
    private static final Pose intakeGarden = poseFactory.of(133.8197, 133.8983, 0);
    private static final Pose flower1 = poseFactory.of(95.2141, 20, 270);
    private static final Pose flower2 = poseFactory.of(132.9813, 94.9733, 0);
    private static final Pose finalLaunch = poseFactory.of(100.0344, 115.7693, 0);
    private static final Pose park = poseFactory.of(139, 43.7628, 180);


    //Paths
    public static Path path1() {
        return Paths.line(startAuto, intakeGarden).constant(0);
    }

    public static Path path2() {
        return Paths.line(intakeGarden, flower1).linear(intakeGarden, flower1);
    }
    public static Path path3() {
        return Paths.line(flower1, flower2).linear(flower1, flower2);
    }

    public static Path path4() {
        return Paths.line(flower2, finalLaunch).constant(finalLaunch);
    }

    public static Path path5() {
        return Paths.line(finalLaunch, park).linear(finalLaunch, park);
    }

}
