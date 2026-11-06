//package org.firstinspires.ftc.teamcode.pedro;
//import static com.pedropathing.api.Paths.*;
//import com.pedropathing.api.Paths;
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.math.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.ivy.Command;
//import com.pedropathing.ivy.Scheduler;
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.commands.Commands.*;
//import static com.pedropathing.ivy.groups.Groups.sequential;
//import static com.pedropathing.ivy.pedro.PedroCommands.follow;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@Autonomous (name = "teste auto")
//public class PathingAuto extends LinearOpMode {
//
//        private Follower follower = Constants.create(hardwareMap);
//
//        private final PoseFactory poseFactory = PoseFactory.degrees();
//
//        private final Pose start = poseFactory.of(88.2129, 133.8563, 0);
//        private final Pose point1 = poseFactory.of(133.8197, 133.8983, 0);
//        private final Pose point2Start = poseFactory.of(133.8197, 133.8983, 270);
//        private final Pose point2 = poseFactory.of(95.2141, 9.2352, 270);
//        private final Pose point3Start = poseFactory.of(95.2141, 9.2352, 270);
//        private final Pose point3 = poseFactory.of(132.9813, 94.9733, 0);
//        private final Pose point4 = poseFactory.of(100.0344, 115.7693, 0);
//        private final Pose point5Start = poseFactory.of(100.0344, 115.7693, 90);
//        private final Pose point5 = poseFactory.of(133.4934, 43.7628, 90);
//
//        // Autonomous routine
//        public Command autoRoutine() {
//            return sequential(
//                    follow(follower, path1()),
//                    follow(follower, path2()),
//                    follow(follower, path3()),
//                    follow(follower, path4()),
//                    follow(follower, path5())
//            );
//        }
//
//    public void runOpMode() {
//            Scheduler.reset();
//            follower = Constants.create(hardwareMap);
//            follower.setPose(start);
//            follower.update();
//
//            waitForStart();
//            schedule(autoRoutine());
//
//            while (opModeIsActive()) {
//                follower.update();
//                Scheduler.execute();
//
//                telemetry.addData("x", follower.pose().x());
//                telemetry.addData("y", follower.pose().y());
//                telemetry.addData("heading", follower.pose().heading());
//
//                if (follower.currentPath() != null) {
//                    telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
//                    telemetry.addData("Path number", follower.pathIndex());
//                }
//
//                telemetry.update();
//            }
//        }
//
//        public Path path1() {
//            return Paths.line(start, point1).linear(start, point1);
//        }
//
//        public Path path2() {
//            return Paths.line(point2Start, point2).linear(point2Start, point2);
//        }
//
//        public Path path3() {
//            return Paths.line(point3Start, point3).linear(point3Start, point3);
//        }
//
//        public Path path4() {
//            return Paths.line(point3, point4).constant(point4);
//        }
//
//        public Path path5() {
//            return Paths.line(point5Start, point5).linear(point5Start, point5);
//        }
//    }
//
