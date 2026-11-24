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
            c.frontRightName.set("FRight");
            c.backLeftName.set("BLeft");
            c.backRightName.set("BRight");
            c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // TODO: replace with measured values from PinpointTuner (in inches).
        c.xPodOffset.set(0.777365616926058);
        c.yPodOffset.set(1.0449010743869571);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.29857011351930807);
                Controller secondaryTranslationalForward = Controller.proportional(0.11031367900501926);
                Controller primaryTranslationalLateral = Controller.proportional(0.4646525822280962);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17167671338762036);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01112398256437511));
                c.brake.set(Controller.proportionalFeedforward(0.009455385179718844));

                c.headingFeedback.set(Controller.proportional(5.659141239186685));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04727937079077097, 0.0074764496083180405));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10293646950535702, 0.07696009176079538));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0017170927494432938, 0.0017775165049763245));

                c.maxAchievableForwardVelocity.set(88.05683533344252);
                c.maxAchievableStrafeVelocity.set(70.07649012262377);
                c.naturalForwardDeceleration.set(26.795129449000044);
                c.naturalStrafeDeceleration.set(64.84571489271187);
            }
    );

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
