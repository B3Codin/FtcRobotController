package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Emily's field-relative drive with intake, turret, and flywheels. */
@TeleOp(name = "JackbotsTeleOp")
public class JackbotsTeleOp extends LinearOpMode {
    // Negative power preserves the front flywheel's spin direction.
    private static final double FLY_FRONT_POWER = -1.0;
    private static final double FLY_BACK_TARGET_RPM = -2100.0;

    // Turret power magnitude (0.0-1.0): LB applies positive, RB applies negative.
    private static final double TURRET_POWER = 0.2;

    // TODO: replace all four placeholders with measured up/down positions.
    private static final double INTAKE_LEFT_UP_POSITION = 0.73;
    private static final double INTAKE_RIGHT_UP_POSITION = 0.00;
    private static final double INTAKE_LEFT_DOWN_POSITION = 0.64;
    private static final double INTAKE_RIGHT_DOWN_POSITION = 0.46;

    // REV Hub velocity gains for the back flywheel. I and D are held at zero.
    private static final double FLY_BACK_P = 100;
    private static final double FLY_BACK_F = 11.211;

    // goBILDA Yellow Jacket front gearbox ratio (nominally 5.2:1).
    private static final double FLY_FRONT_MOTOR_GEARING = 1.0 + 46.0 / 11.0;
    private static final double FLY_FRONT_MOTOR_MAX_RPM = 1150.0;
    private static final double FLY_BACK_MOTOR_GEARING = 1.0;
    private static final double FLY_BACK_MOTOR_MAX_RPM = 6000.0;
    private static final double FLYWHEEL_TICKS_PER_MOTOR_REV = 28.0;
    // Flywheel revolutions per gearbox output revolution.
    private static final double FLYWHEEL_RATIO = 1.0;
    private static final double HEADING_OFFSET_DEGREES = 0.0;
    // Matches the tested direction in RollerTest; the right motor is reversed.
    private static final double INTAKE_ROLLER_POWER = -1.0;
    // Triggers act as buttons: above this threshold runs the rollers at full power.
    private static final double ROLLER_TRIGGER_THRESHOLD = 0.10;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor rollLeft, rollRight;
    private DcMotorEx flyFront, flyBack;
    private Servo intakeLeft, intakeRight;
    private CRServo leftTurretServo, rightTurretServo;
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "FLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FRight");
        backLeft = hardwareMap.get(DcMotor.class, "BLeft");
        backRight = hardwareMap.get(DcMotor.class, "BRight");
        rollLeft = hardwareMap.get(DcMotor.class, "rollLeft");
        rollRight = hardwareMap.get(DcMotor.class, "rollRight");
        flyFront = hardwareMap.get(DcMotorEx.class, "flyFront");
        flyBack = hardwareMap.get(DcMotorEx.class, "flyBack");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        // Keep the configuration names (including spelling) from CRServosTest.
        leftTurretServo = hardwareMap.get(CRServo.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(CRServo.class, "rightTurretServo");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        DcMotor[] allMotors = {
                frontLeft, frontRight, backLeft, backRight,
                rollLeft, rollRight, flyFront, flyBack
        };

        try {
            leftTurretServo.setPower(0.0);
            rightTurretServo.setPower(0.0);
            // Same drive motor names and directions as Emily.
            configurePowerMotor(frontLeft, DcMotor.Direction.FORWARD);
            configurePowerMotor(backLeft, DcMotor.Direction.FORWARD);
            configurePowerMotor(frontRight, DcMotor.Direction.REVERSE);
            configurePowerMotor(backRight, DcMotor.Direction.REVERSE);
            configurePowerMotor(rollLeft, DcMotor.Direction.FORWARD);
            configurePowerMotor(rollRight, DcMotor.Direction.REVERSE);
            configureFlywheel(flyFront, FLY_FRONT_MOTOR_MAX_RPM, FLY_FRONT_MOTOR_GEARING,
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            configureFlywheel(flyBack, FLY_BACK_MOTOR_MAX_RPM, FLY_BACK_MOTOR_GEARING,
                    DcMotor.RunMode.RUN_USING_ENCODER);
            flyBack.setVelocityPIDFCoefficients(FLY_BACK_P, 0.0, 0.0, FLY_BACK_F);
            configurePinpoint();

            telemetry.setMsTransmissionInterval(100);
            telemetry.addLine("Left stick: field-relative drive | Right stick X: turn");
            telemetry.addLine("A: reset X/Y (preserve heading) | LT: intake | RT: outtake");
            telemetry.addLine("Hold LB: turret positive | Hold RB: turret negative");
            telemetry.addData("Left turret connection", leftTurretServo.getConnectionInfo());
            telemetry.addData("Right turret connection", rightTurretServo.getConnectionInfo());
            telemetry.addLine("X: toggle flywheels on/off | Y: toggle intake up/down");
            telemetry.addLine("Intake starts DOWN; positions are placeholders. First Y commands UP.");
            telemetry.addLine("Keep robot still during INIT for odometry calibration.");
            telemetry.update();
            while (opModeInInit()) {
                odo.update();
                Pose2D pose = odo.getPosition();
                telemetry.addLine("INIT: keep robot still until odometry calibration finishes.");
                telemetry.addData("Odometry status", odo.getDeviceStatus());
                telemetry.addData("Distance (m) X", "%.3f", pose.getX(DistanceUnit.METER));
                telemetry.addData("Distance (m) Y", "%.3f", pose.getY(DistanceUnit.METER));
                telemetry.addData("Heading (Degrees)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;

            intakeLeft.setPosition(INTAKE_LEFT_DOWN_POSITION);
            intakeRight.setPosition(INTAKE_RIGHT_DOWN_POSITION);

            boolean previousA = false, previousX = false, previousY = false;
            boolean flywheelsEnabled = false;
            // Match the down positions commanded at START. First Y press goes UP.
            boolean intakeUp = false;
            double frontTicksPerSecondPerRPM = flyFront.getMotorType().getTicksPerRev()
                    / (60.0 * FLYWHEEL_RATIO);
            double backTicksPerSecondPerRPM = flyBack.getMotorType().getTicksPerRev()
                    / (60.0 * FLYWHEEL_RATIO);

            while (opModeIsActive()) {
                boolean a = gamepad1.a;
                boolean x = gamepad1.x;
                boolean y = gamepad1.y;
                boolean leftBumper = gamepad1.left_bumper;
                boolean rightBumper = gamepad1.right_bumper;
                boolean leftTrigger = gamepad1.left_trigger > ROLLER_TRIGGER_THRESHOLD;
                boolean rightTrigger = gamepad1.right_trigger > ROLLER_TRIGGER_THRESHOLD;

                odo.update();
                Pose2D pose = odo.getPosition();
                if (a && !previousA) {
                    // Reset distance once per press without changing field-drive alignment
                    // or recalibrating the IMU while the robot could be moving.
                    pose = new Pose2D(DistanceUnit.METER, 0.0, 0.0,
                            AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS));
                    odo.setPosition(pose);
                }

                double heading = pose.getHeading(AngleUnit.RADIANS)
                        + Math.toRadians(HEADING_OFFSET_DEGREES);
                double fieldForward = -gamepad1.left_stick_y;
                double fieldRight = gamepad1.left_stick_x;
                double robotForward = fieldForward * Math.cos(heading)
                        - fieldRight * Math.sin(heading);
                double robotRight = fieldForward * Math.sin(heading)
                        + fieldRight * Math.cos(heading);
                drive(robotForward, robotRight, gamepad1.right_stick_x);

                // Both triggers or neither trigger stops the rollers.
                double rollerPower = 0.0;
                if (leftTrigger && !rightTrigger) {
                    rollerPower = INTAKE_ROLLER_POWER;
                } else if (rightTrigger && !leftTrigger) {
                    rollerPower = -INTAKE_ROLLER_POWER;
                }
                rollLeft.setPower(rollerPower);
                rollRight.setPower(rollerPower);

                // Both turret CR servos run together, as in CRServosTest.
                double turretPower = 0.0;
                if (leftBumper && !rightBumper) {
                    turretPower = TURRET_POWER;
                } else if (rightBumper && !leftBumper) {
                    turretPower = -TURRET_POWER;
                }
                leftTurretServo.setPower(turretPower);
                rightTurretServo.setPower(turretPower);

                if (x && !previousX) {
                    flywheelsEnabled = !flywheelsEnabled;
                }
                double frontFlywheelPower = flywheelsEnabled ? FLY_FRONT_POWER : 0.0;
                flyFront.setPower(frontFlywheelPower);
                if (flywheelsEnabled) {
                    flyBack.setVelocity(FLY_BACK_TARGET_RPM * backTicksPerSecondPerRPM);
                } else {
                    flyBack.setPower(0.0);
                }

                if (y && !previousY) {
                    intakeUp = !intakeUp;
                    intakeLeft.setPosition(intakeUp
                            ? INTAKE_LEFT_UP_POSITION : INTAKE_LEFT_DOWN_POSITION);
                    intakeRight.setPosition(intakeUp
                            ? INTAKE_RIGHT_UP_POSITION : INTAKE_RIGHT_DOWN_POSITION);
                }

                telemetry.addData("Distance (m) X", "%.3f", pose.getX(DistanceUnit.METER));
                telemetry.addData("Distance (m) Y", "%.3f", pose.getY(DistanceUnit.METER));
                telemetry.addData("Heading (Degrees)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Turret LB / RB", "%b / %b", leftBumper, rightBumper);
                telemetry.addData("Turret commanded power (both servos)", "%.2f", turretPower);
                telemetry.addData("flyFront power", "%.2f", frontFlywheelPower);
                telemetry.addData("flyBack Target rpm", "%.0f", FLY_BACK_TARGET_RPM);
                telemetry.addData("flyFront rpm", "%.1f", flyFront.getVelocity() / frontTicksPerSecondPerRPM);
                telemetry.addData("flyBack rpm", "%.1f", flyBack.getVelocity() / backTicksPerSecondPerRPM);
                telemetry.update();

                previousA = a;
                previousX = x;
                previousY = y;
                idle();
            }
        } finally {
            leftTurretServo.setPower(0.0);
            rightTurretServo.setPower(0.0);
            for (DcMotor motor : allMotors) {
                motor.setPower(0.0);
            }
        }
    }

    private static void configurePowerMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setPower(0.0);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static void configureFlywheel(DcMotorEx motor, double maxRPM, double gearing,
                                          DcMotor.RunMode runMode) {
        motor.setPower(0.0);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorConfigurationType type = motor.getMotorType().clone();
        type.setMaxRPM(maxRPM);
        type.setTicksPerRev(FLYWHEEL_TICKS_PER_MOTOR_REV * gearing);
        type.setGearing(gearing);
        motor.setMotorType(type);
        motor.setMode(runMode);
    }

    private void configurePinpoint() {
        // Same measured offsets, pod type, and encoder directions as Emily.
        odo.setOffsets(0.4161412697138749, -2.594972445270208, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    private void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;
        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        frontLeft.setPower(frontLeftPower / maxPower);
        frontRight.setPower(frontRightPower / maxPower);
        backLeft.setPower(backLeftPower / maxPower);
        backRight.setPower(backRightPower / maxPower);
    }
}
