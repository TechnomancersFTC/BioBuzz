package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class paths{


    //Pontos
    private static final PoseFactory poseFactory = PoseFactory.degrees();
    public static final Pose startPoseteleop = poseFactory.of(9.4, 7.8, 90);
    public static final Pose start = poseFactory.of(87.4, 133.8, 270);
    private static final Pose point1 = poseFactory.of(133.8197, 133.8983, 0);
    private static final Pose point2 = poseFactory.of(95.2141, 14, 270);
    private static final Pose point3Start = poseFactory.of(95.2141, 9.2352, 270);
    private static final Pose point3 = poseFactory.of(132.9813, 94.9733, 0);
    private static final Pose point4 = poseFactory.of(100.0344, 115.7693, 0);
    private static final Pose point5Start = poseFactory.of(100.0344, 115.7693, 90);
    private static final Pose park = poseFactory.of(139, 43.7628, 180);


    //Paths
    public static Path path1() {
        return Paths.line(start, point1).constant(0);
    }

    public static Path path2() {
        return Paths.line(point1, point2).linear(point1, point2);
    }

    public static Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public static Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }

    public static Path path5() {
        return Paths.line(point5Start, park).linear(point5Start, park);
    }

}
