package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Initializes Pedro against the configured hardware without commanding movement.
 * Use this after deployment to catch missing device names before running a tuner.
 */
@TeleOp(name = "Pedro: Hardware Smoke Test", group = "Pedro Pathing")
public final class PedroHardwareSmokeTest extends OpMode {
    private Follower follower;
    private String initializationError;

    @Override
    public void init() {
        try {
            follower = Constants.create(hardwareMap);
            follower.setPose(ConstantsPose.START);
        } catch (RuntimeException error) {
            initializationError = error.getClass().getSimpleName() + ": " + error.getMessage();
        }
    }

    @Override
    public void loop() {
        if (initializationError != null) {
            telemetry.addLine("Pedro initialization failed");
            telemetry.addLine(initializationError);
        } else {
            // update() is safe here: the follower is idle and only reports localizer state.
            follower.update();
            telemetry.addData("status", "Pedro initialized (no movement commanded)");
            telemetry.addData("x (in)", "%.2f", follower.pose().x());
            telemetry.addData("y (in)", "%.2f", follower.pose().y());
            telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) follower.stop();
    }

    private static final class ConstantsPose {
        private static final com.pedropathing.math.Pose START =
                new com.pedropathing.math.Pose(0.0, 0.0, 0.0);
    }
}
