package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Single source of truth for the Pedro Pathing hardware and controller setup.
 *
 * <p>The motor and Pinpoint names below match the configuration currently on the
 * robot's Control Hub. The Pinpoint offsets and Foresight values are deliberately
 * conservative starter values; run the tuners before attempting an autonomous path.
 */
public final class Constants {
    private Constants() {}

    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FLeft");
        c.backLeftName.set("BLeft");
        c.frontRightName.set("FRight");
        c.backRightName.set("BRight");

        // Conventional mecanum directions. Confirm with MecanumTuner on the real robot.
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });

    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // TODO: replace with measured values from PinpointTuner (in inches).
        c.xPodOffset.set(0.4161412697138749);
        c.yPodOffset.set(-2.594972445270208);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1620818691414957);
                Controller secondaryTranslationalForward = Controller.proportional(0.05988491974047573);
                Controller primaryTranslationalLateral = Controller.proportional(0.23358970122995915);
                Controller secondaryTranslationalLateral = Controller.proportional(0.08630515297269926);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010537340423932019));
                c.brake.set(Controller.proportionalFeedforward(0.008956739360342216));

                c.headingFeedback.set(Controller.proportional(2.9922545423174602));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.036592316363342835, 0.008344297573493668));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0709874954354331, 0.03115277746484963));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001410768813643856, 0.0019217016263227034));

                c.maxAchievableForwardVelocity.set(90.91486878363857);
                c.maxAchievableStrafeVelocity.set(76.57089363499806);
                c.naturalForwardDeceleration.set(32.68175155574763);
                c.naturalStrafeDeceleration.set(50.03615865785835);
            }
    );

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
