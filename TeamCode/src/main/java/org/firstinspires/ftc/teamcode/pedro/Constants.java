package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
            c.frontLeftName.set("fl");
            c.frontRightName.set("fr");
            c.backLeftName.set("bl");
            c.backRightName.set("br");
            c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.manualBrakeMode.set(Boolean.TRUE);
        });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
            c.name.set("pinpoint");
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            c.xPodOffset.set(0.7903626209168921);
            c.yPodOffset.set(-3.5817586343119467);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
        });
    public static ForesightConfig foresightConfig = new ForesightConfig(
                c -> {
                    Controller primaryTranslationalForward = Controller.proportional(0.2989394532469906);
                    Controller secondaryTranslationalForward = Controller.proportional(0.11045014016545868);
                    Controller primaryTranslationalLateral = Controller.proportional(0.5637385145400674);
                    Controller secondaryTranslationalLateral = Controller.proportional(0.20828631775202033);

                    c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                    c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                    c.coast.set(Controller.proportionalFeedforward(0.015407539077532529));
                    c.brake.set(Controller.proportionalFeedforward(0.013096408215902649));

                    c.headingFeedback.set(Controller.proportional(5.0999267151627));
                    c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04150178552175371, 0.007216163593211382));

                    c.linearBrakeCoefficients.set(Matrix.diag(0.09961843437810043, 0.05580739763433227));
                    c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015226163342220982, 0.002126717246675951));

                    c.maxAchievableForwardVelocity.set(50.78285473650123);
                    c.maxAchievableStrafeVelocity.set(46.17596318886892);
                    c.naturalForwardDeceleration.set(68.70013993929452);
                    c.naturalStrafeDeceleration.set(68.05892464697932);
                }
        );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}