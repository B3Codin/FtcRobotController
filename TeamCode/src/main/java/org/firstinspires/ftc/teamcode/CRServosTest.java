package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/** Tests the turret servos and counter-rotating intake at full power. */
@TeleOp(name = "CRServosTest", group = "Servo")
public class CRServosTest extends LinearOpMode {
    private static final double TEST_POWER = 1.0;
    private static final double INTAKE_POWER = 1.0;

    @Override
    public void runOpMode() {
        // These names must match the continuous rotation servos in the robot configuration.
        CRServo left = hardwareMap.get(CRServo.class, "leftTurrettServo");
        CRServo right = hardwareMap.get(CRServo.class, "rightTurrettServo");
        DcMotor leftIntake = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        DcMotor rightIntake = hardwareMap.get(DcMotor.class, "rightIntakeMotor");

        try {
            left.setPower(0.0);
            right.setPower(0.0);
            leftIntake.setPower(0.0);
            rightIntake.setPower(0.0);
            leftIntake.setDirection(DcMotor.Direction.FORWARD);
            rightIntake.setDirection(DcMotor.Direction.REVERSE);
            leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addLine("Ready: both turret CR servos, full power, same direction.");
            telemetry.addLine("Hold D-pad right (+100%) or left (-100%) to run both.");
            telemetry.addLine("Hold A: intake forward at 100% | Y: intake reverse at 100%.");
            telemetry.addLine("Release each control to stop that mechanism. B stops all.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                double power = 0.0;
                if (!gamepad1.b && gamepad1.dpad_right != gamepad1.dpad_left) {
                    power = gamepad1.dpad_right ? TEST_POWER : -TEST_POWER;
                }

                left.setPower(power);
                right.setPower(power);
                double intakePower = 0.0;
                if (!gamepad1.b && gamepad1.a != gamepad1.y) {
                    intakePower = gamepad1.a ? INTAKE_POWER : -INTAKE_POWER;
                }
                leftIntake.setPower(intakePower);
                rightIntake.setPower(intakePower);

                telemetry.addLine("Both servos run together in the same direction.");
                telemetry.addLine("D-pad right: +100% | left: -100% | B: stop");
                telemetry.addLine("Hold A: intake forward 100% | Y: reverse 100% | A+Y: stop intake");
                telemetry.addData("leftTurrettServo commanded power", "%.2f", power);
                telemetry.addData("rightTurrettServo commanded power", "%.2f", power);
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
}
