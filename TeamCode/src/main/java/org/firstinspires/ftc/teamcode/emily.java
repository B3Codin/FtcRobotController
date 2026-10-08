package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
// poopy butt
/** Field-relative mecanum drive: left stick moves relative to the field, right stick turns. */
@TeleOp(name = "emily")
public class emily extends LinearOpMode {
    // Adjust this if the robot starts pointed at a different angle than the driver's
    // field-forward direction. Positive values rotate the field reference counterclockwise.
    // For example, use 90 if the robot starts facing 90 degrees counterclockwise from field-forward.
    private static final double HEADING_OFFSET_DEGREES = 0.0;

    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor BLeft;
    private DcMotor BRight;
    GoBildaPinpointDriver odo;

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

        // Get a reference to the sensor
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Configure the sensor
        configurePinpoint();

        // Set the initial position. Pinpoint's heading is zeroed by resetPosAndIMU().
        odo.setPosition(new Pose2D(DistanceUnit.CM, 0, 0,AngleUnit.DEGREES, 0));

        telemetry.addLine("Ready: left stick moves field-relative, right stick turns.");
        telemetry.addData("Heading offset (degrees)", HEADING_OFFSET_DEGREES);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pose2D = odo.getPosition();
            double heading = pose2D.getHeading(AngleUnit.RADIANS)
                    + Math.toRadians(HEADING_OFFSET_DEGREES);

            // FTC stick Y is negative when pushed forward.
            double fieldForward = -gamepad1.left_stick_y;
            double fieldRight = gamepad1.left_stick_x;

            // Convert the driver's field-relative command into robot-relative motion.
            // Pinpoint heading is counterclockwise-positive; at +90 degrees, field-forward
            // becomes robot-right.
            double robotForward = fieldForward * Math.cos(heading)
                    - fieldRight * Math.sin(heading);
            double robotRight = fieldForward * Math.sin(heading)
                    + fieldRight * Math.cos(heading);
            drive(robotForward, robotRight, gamepad1.right_stick_x);

            telemetry.addLine("Press A to reset the position");
            if (gamepad1.a) {
                // Reset X/Y while preserving heading so field-centric driving stays aligned.
                odo.setPosition(new Pose2D(DistanceUnit.CM, 0, 0,
                        AngleUnit.RADIANS, pose2D.getHeading(AngleUnit.RADIANS)));
            }

            telemetry.addData("X coordinate (MM)", pose2D.getX(DistanceUnit.MM));
            telemetry.addData("Y coordinate (MM)", pose2D.getY(DistanceUnit.MM));
            telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Field heading used (DEGREES)", Math.toDegrees(heading));

            telemetry.update();
            idle();
        }
    }

    private void configurePinpoint() {

        odo.setOffsets(0.4161412697138749, -2.594972445270208, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

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
