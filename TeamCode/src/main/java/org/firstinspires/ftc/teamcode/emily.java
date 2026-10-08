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
/** Robot-relative mecanum drive: left stick translates, right stick turns. */
@TeleOp(name = "emily")
public class emily extends LinearOpMode {
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

        // Set the location of the robot - this should be the place you are starting the robot from
        odo.setPosition(new Pose2D(DistanceUnit.CM, 0, 0,AngleUnit.DEGREES, 0));

        telemetry.addLine("Ready: left stick moves, right stick turns (robot-relative).");
        telemetry.update();
        waitForStart();

            while (opModeIsActive()) {
                // FTC stick Y is negative when pushed forward.
                drive(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                        gamepad1.right_stick_x);

                telemetry.addLine("Press A to reset the position");
                if(gamepad1.a){
                    // You could use readings from April Tags here to give a new known position to the pinpoint
                    odo.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0));
                }
                odo.update();
                Pose2D pose2D = odo.getPosition();

                telemetry.addData("X coordinate (CM)", pose2D.getX(DistanceUnit.CM));
                telemetry.addData("Y coordinate (CM)", pose2D.getY(DistanceUnit.CM));
                telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));


                telemetry.update();
                idle();
            }

    }

    private void configurePinpoint() {

        odo.setOffsets(0, -130, DistanceUnit.MM);
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
