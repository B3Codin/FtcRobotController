package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;

/** Tests the turret servos together or individually, plus the counter-rotating intake. */
@TeleOp(name = "CRServosTest", group = "Servo")
public class CRServosTest extends LinearOpMode {
    private static double TEST_POWER = 0.5;
    private static final double INTAKE_POWER = 1.0;

    @Override
    public void runOpMode() {
        // These names must match the continuous rotation servos in the robot configuration.
        CRServo left = hardwareMap.get(CRServo.class, "leftTurretServo");
        CRServo right = hardwareMap.get(CRServo.class, "rightTurretServo");
        // Match JackbotsTeleOp/RollerTest; intakeLeft/intakeRight are positional servos.
        DcMotor leftIntake = hardwareMap.get(DcMotor.class, "rollLeft");
        DcMotor rightIntake = hardwareMap.get(DcMotor.class, "rollRight");

        try {
            left.setPower(0.0);
            right.setPower(0.0);
            leftIntake.setPower(0.0);
            rightIntake.setPower(0.0);
            leftIntake.setDirection(DcMotor.Direction.FORWARD);
            rightIntake.setDirection(DcMotor.Direction.REVERSE);
            leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addData("Turret test power", "%.0f%%", TEST_POWER * 100.0);
            telemetry.addLine("Hold D-pad right (+) or left (-) to run both.");
            telemetry.addLine("Also hold LB: left only | RB: right only | LB+RB: stop turret.");
            telemetry.addLine("For individual tests, disengage the servos from the turret first.");
            telemetry.addData("Left connection", left.getConnectionInfo());
            telemetry.addData("Right connection", right.getConnectionInfo());
            telemetry.addLine("Hold A: intake forward at 100% | Y: intake reverse at 100%.");
            telemetry.addLine("Release each control to stop that mechanism. B stops all.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                double power = 0.0;
                if (!gamepad1.b && gamepad1.dpad_right != gamepad1.dpad_left) {
                    power = gamepad1.dpad_right ? TEST_POWER : -TEST_POWER;
                }

                if(gamepad1.dpad_down){
                    TEST_POWER -= 0.1;
                } else if (gamepad1.dpad_up){
                    TEST_POWER += 0.1;
                }

                double leftPower = gamepad1.right_bumper ? 0.0 : power;
                double rightPower = gamepad1.left_bumper ? 0.0 : power;
                left.setPower(leftPower);
                right.setPower(rightPower);
                double intakePower = 0.0;
                if (!gamepad1.b && gamepad1.a != gamepad1.y) {
                    intakePower = gamepad1.a ? INTAKE_POWER : -INTAKE_POWER;
                }
                leftIntake.setPower(intakePower);
                rightIntake.setPower(intakePower);

                telemetry.addData("Turret test power", "%.0f%%", TEST_POWER * 100.0);
                telemetry.addLine("D-pad right: + | left: - | B: stop all");
                telemetry.addLine("Hold LB: left only | RB: right only | LB+RB: stop turret");
                telemetry.addLine("Hold A: intake forward 100% | Y: reverse 100% | A+Y: stop intake");
                telemetry.addData("D-pad right / left / B", "%b / %b / %b",
                        gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.b);
                telemetry.addData("leftTurretServo commanded power", "%.2f", leftPower);
                telemetry.addData("rightTurretServo commanded power", "%.2f", rightPower);
                telemetry.addData("Left connection", left.getConnectionInfo());
                telemetry.addData("Right connection", right.getConnectionInfo());
                addPwmTelemetry("Left", left);
                addPwmTelemetry("Right", right);
                telemetry.addLine("Commands/PWM status do not measure servo motion or supply voltage.");
                telemetry.addData("leftIntakeMotor commanded power (direction FORWARD)", "%.2f", intakePower);
                telemetry.addData("rightIntakeMotor commanded power (direction REVERSE)", "%.2f", intakePower);
                telemetry.update();
                sleep(20);
            }
        } finally {
            left.setPower(0.0);
            right.setPower(0.0);
            leftIntake.setPower(0.0);
            rightIntake.setPower(0.0);
        }
    }

    private void addPwmTelemetry(String label, CRServo servo) {
        if (servo instanceof PwmControl) {
            PwmControl pwm = (PwmControl) servo;
            telemetry.addData(label + " PWM enabled", pwm.isPwmEnabled());
            telemetry.addData(label + " commanded pulse (us)", "%.0f", pwm.getPulseWidth());
        } else {
            telemetry.addData(label + " PWM diagnostics", "unavailable on this controller");
        }
    }
}
