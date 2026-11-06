//package org.firstinspires.ftc.teamcode.Autos;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.pedropathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedro.Constants;
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.math.Pose;
//import static com.pedropathing.api.Paths.*;
//import com.pedropathing.paths.Path;
//import com.pedropathing.ivy.Scheduler;
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.pedro.PedroCommands.follow;
//import com.pedropathing.ivy.Command;
//import com.pedropathing.ivy.Scheduler;
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.groups.Groups.sequential;
//import static com.pedropathing.ivy.pedro.PedroCommands.follow;
//    @Autonomous
//    public class BioAuto extends OpMode {
//        private Follower follower;
//        private final PoseFactory p = PoseFactory.degrees();
//        private Path park() {
//            return line(start, point5).linear(start, point5);
//        }
//        private final Pose start = p.of(88.2129, 133.8563, 0);
//        private final Pose point1 = p.of(133.8197, 133.8983, 0);
//        private final Pose point2Start = p.of(133.8197, 133.8983, 270);
//        private final Pose point2 = p.of(95.2141, 9.2352, 270);
//        private final Pose point3Start = p.of(95.2141, 9.2352, 270);
//        private final Pose point3 = p.of(132.9813, 94.9733, 0);
//        private final Pose point4 = p.of(100.0344, 115.7693, 0);
//        private final Pose point5Start = p.of(100.0344, 115.7693, 90);
//        private final Pose point5 = p.of(133.4934, 43.7628, 90);
//
//        @Override
//        public void init() {
//            follower = Constants.create(hardwareMap);
//            follower.setPose(start);
//        }
//
//        @Override
//        public void start() {
//            schedule(follow(follower, park()));
//        }
//
//        @Override
//        public void loop() {
//            follower.update();
//            Scheduler.execute();
//        }
//    }
//
