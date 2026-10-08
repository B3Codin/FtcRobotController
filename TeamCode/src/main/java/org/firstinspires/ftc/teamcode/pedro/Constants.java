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

    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.forwardTranslational.set(
                Controller.piecewise(Controller.proportional(0.10))
                        .put(2.5, Controller.proportional(0.30)));
        c.strafeTranslational.set(
                Controller.piecewise(Controller.proportional(0.10))
                        .put(2.5, Controller.proportional(0.30)));
        c.headingFeedback.set(Controller.proportional(1.0));
        // Nonzero placeholders keep the un-tuned follower numerically well-defined.
        // Replace every Foresight value by running ForesightTuner before pathing.
        c.brake.set(Controller.proportionalFeedforward(0.01));
        c.coast.set(Controller.proportionalFeedforward(0.01));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05, 0.005));
        c.linearBrakeCoefficients.set(Matrix.diag(0.10, 0.10));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.001, 0.001));
        c.maxAchievableForwardVelocity.set(50.0);
        c.maxAchievableStrafeVelocity.set(50.0);
        c.naturalForwardDeceleration.set(50.0);
        c.naturalStrafeDeceleration.set(50.0);
    });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
