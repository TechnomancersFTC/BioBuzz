package org.firstinspires.ftc.teamcode.Autos;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous
public class PlanB extends OpMode {
    private DcMotor shooter, intake, intake2;
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose start = p.of(93.4907, 133.7576, 270);
    private final Pose path1 = p.of(135.3492, 36.364, 180);

    private Path startToScore() {
        return line(start, path1).linear(start, path1);
    }
    private Path park(){
        return line(start, path1).linear(start, path1);
    }
    private Command autoRoutine() {
        return sequential(
                waitMs(4000),
                instant(() -> intake.setPower(1)),
                instant(() -> intake2.setPower(1)),
                waitMs(100),
                instant(() -> intake.setPower(0)),
                instant(() -> intake2.setPower(0)),
                waitMs(1000),
                instant(() -> intake.setPower(1)),
                instant(() -> intake2.setPower(1)),
                waitMs(100),
                instant(() -> intake.setPower(0)),
                instant(() -> intake2.setPower(0)),
                waitMs(2500),
                instant(() -> intake2.setPower(1)),
                instant(() -> intake.setPower(1)),
                waitMs(100),
                instant(() -> intake.setPower(0)),
                instant(() -> intake2.setPower(0)),
                waitMs(3000),
                instant(() -> intake.setPower(1)),
                instant(() -> intake2.setPower(1)),
                waitMs(100),
                instant(() -> intake.setPower(0)),
                instant(() -> intake2.setPower(0)),
                waitMs(500),
                follow(follower, startToScore()),
                follow(follower, park()),
                instant(() -> shooter.setPower(0))
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        shooter = hardwareMap.get(DcMotor.class,"shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        shooter.setPower(-0.625);
        follower.update();
        Scheduler.execute();
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();

    }}