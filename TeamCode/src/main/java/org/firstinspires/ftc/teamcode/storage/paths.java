package org.firstinspires.ftc.teamcode.storage;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class paths{


    private static final PoseFactory poseFactory = PoseFactory.degrees();
    public static final Pose startPoseteleop = poseFactory.of(9.4, 7.8, 90);
    public static final Pose startAuto = poseFactory.of(70.75, 133.8, 270);
    public static final Pose middle = poseFactory.of(100, 135, 0);
    private static final Pose intakeGarden = poseFactory.of(117, 135, 0);
    private static final Pose flowerStop = poseFactory.of(95, 35, 270);
    private static final Pose flower1 = poseFactory.of(84, 15, 270);
    private static final Pose flower2 = poseFactory.of(116, 98, 0);
    private static final Pose flowerStop2 = poseFactory.of(95, 98, 0);
    private static final Pose finalLaunch = poseFactory.of(87, 125, 0);
    public static final Pose park = poseFactory.of(125, 40, 180);


    public static Path path1() {
        return Paths.line(startAuto, intakeGarden).constant(0)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.75));
    }

    public static Path middle(){
        return Paths.line(intakeGarden, middle).constant(0)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.75));
    }
    public static Path middleBack(){
        return Paths.line(middle, intakeGarden).constant(0)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.75));
    }

    public static Path path2() {
        return Paths.line(intakeGarden, flowerStop).linear(intakeGarden, flowerStop)
                .with(Constants.foresightConfig.maxPathSpeed.at(1.0));
    }

    public static Path flowerTake() {
        return Paths.line(flowerStop, flower1).constant(flowerStop)
                .with(Constants.foresightConfig.maxPathSpeed.at(1.0));
    }

    public static Path flowerBack(){
        return Paths.line(flower1, flowerStop).constant(flowerStop)
                .with(Constants.foresightConfig.maxPathSpeed.at(1.0));
    }
    public static Path path3() {
        return Paths.line(flowerStop, flowerStop2).linear(flowerStop, flowerStop2)
                .with(Constants.foresightConfig.maxPathSpeed.at(1.0));
    }

    public static Path flowerTake2(){
        return Paths.line(flowerStop2, flower2).constant(0)
                .with(Constants.foresightConfig.maxPathSpeed.at(1.0));
    }

    public static Path flowerBack2(){
        return Paths.line(flower2, flowerStop2).constant(flowerStop2)
                .with(Constants.foresightConfig.maxPathSpeed.at(1.0));
    }

    public static Path path4() {
        return Paths.line(flowerStop2, finalLaunch).constant(finalLaunch)
                .with(Constants.foresightConfig.maxPathSpeed.at(1.0));
    }

    public static Path path5() {
        return Paths.line(finalLaunch, park).linear(finalLaunch, park)
                .with(Constants.foresightConfig.maxPathSpeed.at(1.0));

    }

}