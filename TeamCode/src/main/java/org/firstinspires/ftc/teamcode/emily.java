package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/** Robot-relative mecanum drive: left stick translates, right stick turns. */
@TeleOp(name = "emily")
public class emily extends LinearOpMode {
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor BLeft;
    private DcMotor BRight;

    @Override
    public void runOpMode() {
        FLeft = hardwareMap.get(DcMotor.class, "FLeft");
        FRight = hardwareMap.get(DcMotor.class, "FRight");
        BLeft = hardwareMap.get(DcMotor.class, "BLeft");
        BRight = hardwareMap.get(DcMotor.class, "BRight");

        // Match the directions currently configured in pedro/Constants.java.
        // Confirm physical wheel directions with the robot lifted.
        FLeft.setDirection(DcMotor.Direction.FORWARD);
        BLeft.setDirection(DcMotor.Direction.FORWARD);
        FRight.setDirection(DcMotor.Direction.REVERSE);
        BRight.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[] {FLeft, FRight, BLeft, BRight}) {
            motor.setPower(0.0);
            // Joystick power control does not require motor encoder feedback.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Ready: left stick moves, right stick turns (robot-relative).");
        telemetry.update();
        waitForStart();

            while (opModeIsActive()) {
                // FTC stick Y is negative when pushed forward.
//                drive(-gamepad1.left_stick_y, gamepad1.left_stick_x,
//                        gamepad1.right_stick_x);


                    FLeft.setPower(1);
                    FRight.setPower(1);
                    BLeft.setPower(1);
                    BRight.setPower(1);


                telemetry.addLine("Robot-relative: left stick moves, right stick turns.");
                telemetry.addData("Front Left / Right power", "%.2f / %.2f",
                        FLeft.getPower(), FRight.getPower());
                telemetry.addData("Back Left / Right power", "%.2f / %.2f",
                        BLeft.getPower(), BRight.getPower());
                telemetry.update();
                idle();
            }

    }

    public void drive(double forward, double right, double rotate) {
        // Positive rotate commands a clockwise turn. Motor direction settings
        // handle mirrored mounting; do not invert the right side again here.
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;

        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        FLeft.setPower(frontLeftPower / maxPower);
        FRight.setPower(frontRightPower / maxPower);
        BLeft.setPower(backLeftPower / maxPower);
        BRight.setPower(backRightPower / maxPower);
    }
}
