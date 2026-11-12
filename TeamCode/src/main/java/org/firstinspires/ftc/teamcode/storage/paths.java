package org.firstinspires.ftc.teamcode.storage;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class paths {

    public static volatile PoseFactory poseFactory = PoseFactory.degrees();

    public static final Pose startPoseteleop = poseFactory.of(9.4, 7.8, 90);
    public static final Pose startAuto = poseFactory.of(70.75, 133.8, 270);

    public static volatile Pose intakeGarden = poseFactory.of(117, 135, 0);
    public static volatile Pose flowerStop = poseFactory.of(95, 35, 270);
    public static volatile Pose flower1 = poseFactory.of(84, 14.5, 270);
    public static volatile Pose flower2 = poseFactory.of(117, 98, 0);
    public static volatile Pose flowerStop2 = poseFactory.of(113, 98, 0);
    public static volatile Pose finalLaunch = poseFactory.of(87, 125, 0);
    public static volatile Pose park = poseFactory.of(134, 43.7628, 180);

    private static Pose lastAutoPose = null;
    private static boolean hasAutoPose = false;

    public static void saveAutoPose(Pose pose) {
        lastAutoPose = new Pose(
                pose.x(),
                pose.y(),
                pose.heading()
        );

        hasAutoPose = true;
    }

    public static Pose getTeleopStartPose() {
        if (hasAutoPose && lastAutoPose != null) {
            return lastAutoPose;
        }

        return startPoseteleop;
    }

    public static void resetAutoPose() {
        lastAutoPose = null;
        hasAutoPose = false;
    }
    public static Path path1() {
        return Paths.line(startAuto, intakeGarden).constant(0)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.75));
    }

    public static Path path2() {
        return Paths.line(intakeGarden, flowerStop).linear(intakeGarden, flowerStop);
    }

    public static Path flowerTake() {
        return Paths.line(flowerStop, flower1).constant(flowerStop);
    }

    public static Path flowerBack() {
        return Paths.line(flower1, flowerStop).constant(flowerStop);
    }

    public static Path path3() {
        return Paths.line(flowerStop, flowerStop2).linear(flowerStop, flowerStop2);
    }

    public static Path flowerTake2() {
        return Paths.line(flowerStop2, flower2).constant(0);
    }

    public static Path flowerBack2() {
        return Paths.line(flower2, flowerStop2).constant(flowerStop2);
    }

    public static Path path4() {
        return Paths.line(flowerStop2, finalLaunch).constant(finalLaunch);
    }

    public static Path path5() {
        return Paths.line(finalLaunch, park).linear(finalLaunch, park);
    }
}