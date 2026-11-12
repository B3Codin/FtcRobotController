package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/** Adjusts one positional intake servo at a time using gamepad 1. */
@TeleOp(name = "Intake Servo Test", group = "Servo")
public class IntakeServoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        String[] names = {"intakeLeft", "intakeRight"};
        Servo[] servos = {
                hardwareMap.get(Servo.class, names[0]),
                hardwareMap.get(Servo.class, names[1])
        };
        // getPosition reports the last command, not the measured physical position.
        // Do not send a new position until the driver requests an adjustment.
        double[] positions = {servos[0].getPosition(), servos[1].getPosition()};
        int selected = 0;
        boolean fineStep = false;

        telemetry.addLine("Gamepad 1: A switches servo | X switches step (0.10 / 0.01)");
        telemetry.addLine("D-pad up/down: increase/decrease selected servo position");
        telemetry.addLine("Each press adjusts once. Positions are limited to 0.00-1.00.");
        telemetry.addLine("No new position is commanded until you press D-pad up/down.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        boolean previousA = gamepad1.a;
        boolean previousX = gamepad1.x;
        boolean previousUp = gamepad1.dpad_up;
        boolean previousDown = gamepad1.dpad_down;

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean x = gamepad1.x;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            if (a && !previousA) {
                selected = 1 - selected;
            }
            if (x && !previousX) {
                fineStep = !fineStep;
            }
            double step = fineStep ? 0.01 : 0.1;

            // Ignore opposing inputs; holding a button does not repeat movement.
            if (up != down && ((up && !previousUp) || (down && !previousDown))) {
                double next = positions[selected] + (up ? step : -step);
                positions[selected] = Math.max(0.0, Math.min(1.0,
                        Math.round(next * 100.0) / 100.0));
                servos[selected].setPosition(positions[selected]);
            }

            telemetry.addData("Selected servo", names[selected]);
            telemetry.addData("Step size", "%.2f", step);
            telemetry.addData("intakeLeft commanded position", "%.2f", positions[0]);
            telemetry.addData("intakeRight commanded position", "%.2f", positions[1]);
            telemetry.addLine("A: switch servo | X: toggle 0.10 / 0.01 step");
            telemetry.addLine("D-pad up: increase | D-pad down: decrease (one step per press)");
            telemetry.update();

            previousA = a;
            previousX = x;
            previousUp = up;
            previousDown = down;
            sleep(20);
        }
    }
}
