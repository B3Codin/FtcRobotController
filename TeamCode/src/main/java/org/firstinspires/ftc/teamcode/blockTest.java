package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/** Manually finds the open and closed positions for the shooting blocker. */
@TeleOp(name = "blockTest", group = "Servo")
public class blockTest extends LinearOpMode {
    private static final double[] STEP_SIZES = {0.001, 0.01, 0.1};

    @Override
    public void runOpMode() {
        Servo blockServo = hardwareMap.get(Servo.class, "blockServo");
        // This is the last commanded position, not a physical position measurement.
        // If no command is available, use 0.5 as the adjustment starting point.
        // Do not move the servo until the driver presses D-pad up/down.
        double position = blockServo.getPosition();
        if (Double.isNaN(position) || Double.isInfinite(position)) {
            position = 0.5;
        }
        position = Math.max(-1.0, Math.min(1.0, position));
        int stepIndex = 2;

        showTelemetry(position, STEP_SIZES[stepIndex]);
        waitForStart();
        if (isStopRequested()) return;

        boolean previousLb = gamepad1.left_bumper;
        boolean previousRb = gamepad1.right_bumper;
        boolean previousUp = gamepad1.dpad_up;
        boolean previousDown = gamepad1.dpad_down;

        while (opModeIsActive()) {
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            // Each press adjusts once; opposing buttons cancel each other.
            if (lb != rb) {
                if (lb && !previousLb) {
                    stepIndex = Math.max(0, stepIndex - 1);
                } else if (rb && !previousRb) {
                    stepIndex = Math.min(STEP_SIZES.length - 1, stepIndex + 1);
                }
            }

            double step = STEP_SIZES[stepIndex];
            if (up != down && ((up && !previousUp) || (down && !previousDown))) {
                double next = position + (up ? step : -step);
                position = Math.max(0.0, Math.min(1.0,
                        Math.round(next * 1000.0) / 1000.0));
                blockServo.setPosition(position);
            }

            showTelemetry(position, step);
            previousLb = lb;
            previousRb = rb;
            previousUp = up;
            previousDown = down;
            sleep(20);
        }
    }

    private void showTelemetry(double position, double step) {
        telemetry.addData("blockServo position (commanded)", "%.3f", position);
        telemetry.addData("Step size", "%.3f", step);
        telemetry.addData("Button guide",
                "Gamepad 1 | LB: smaller step | RB: larger step | "
                        + "D-pad up: increase | D-pad down: decrease | One change per press");
        telemetry.update();
    }
}
