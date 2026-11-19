package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Field-relative driving and an encoder-based turret test; field aiming is not enabled. */
@TeleOp(name = "TurretAimTest")
public class TurretAimTest extends LinearOpMode {
    // Future field aiming target, in meters relative to the odometry origin. UNUSED for now.
    private static final double FIELD_TARGET_X_METERS = 1.0;
    private static final double FIELD_TARGET_Y_METERS = 1.0;

    // REV Through Bore Encoder V2 in Expansion Hub encoder port 0, shared with
    // FRight's motor channel. Its quadrature output is 8192 counts per shaft revolution.
    // Read through frontRight; never STOP_AND_RESET_ENCODER on this shared drive port.
    private static final double ENCODER_TICKS_PER_REVOLUTION = 8192.0;
    // FRight's REVERSE motor direction also reverses getCurrentPosition(). Set this
    // to -1 if positive turret motion makes that returned encoder count decrease.
    private static final int ENCODER_SIGN = 1;
    // Configure limitSwitch as a Digital Device on the Control Hub 0-1 connector.
    // The REV magnetic switch pulls both signal pins low when a magnet is detected.
    private static final String ZERO_SWITCH_NAME = "limitSwitch";
    private static final boolean ZERO_SWITCH_ACTIVE_LOW = true;
    private static final double ENCODER_REVOLUTIONS_PER_TURRET_DEGREE = 3.75;
    private static final double TEST_TARGET_DEGREES = 90.0;
    private static final double TURRET_LIMIT_DEGREES = 180.0;
    // Same positive power convention as JackbotsTeleOp: both servos get +0.2.
    private static final double TURRET_TEST_POWER = 0.2;
    private static final double ENCODER_NO_PROGRESS_TIMEOUT_SECONDS = 1.0;
    private static final double HEADING_OFFSET_DEGREES = 0.0;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor turretEncoder;
    private DigitalChannel limitSwitch;
    private CRServo leftTurretServo, rightTurretServo;
    private GoBildaPinpointDriver odo;
    private int turretZeroTicks;
    private boolean magnetDetected;
    private boolean magnetZeroEstablished;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "FLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FRight");
        backLeft = hardwareMap.get(DcMotor.class, "BLeft");
        backRight = hardwareMap.get(DcMotor.class, "BRight");
        leftTurretServo = hardwareMap.get(CRServo.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(CRServo.class, "rightTurretServo");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        try {
            setTurretPower(0.0);
            configureDriveMotor(frontLeft, DcMotor.Direction.FORWARD);
            configureDriveMotor(backLeft, DcMotor.Direction.FORWARD);
            configureDriveMotor(frontRight, DcMotor.Direction.REVERSE);
            configureDriveMotor(backRight, DcMotor.Direction.REVERSE);
            configurePinpoint();

            turretEncoder = frontRight;
            limitSwitch = hardwareMap.get(DigitalChannel.class, ZERO_SWITCH_NAME);
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
            // Retain the straight-ahead INIT reference until the magnet is detected.
            // Only change our offset: a hardware reset would interrupt FRight driving.
            turretZeroTicks = turretEncoder.getCurrentPosition();
            updateMagnetZero();
            boolean encoderReady = ENCODER_SIGN == 1 || ENCODER_SIGN == -1;

            telemetry.setMsTransmissionInterval(100);
            while (opModeInInit()) {
                odo.update();
                updateMagnetZero();
                telemetry.addLine("INIT: keep robot still until odometry calibration finishes.");
                telemetry.addLine("Start turret STRAIGHT; magnet detection corrects zero automatically.");
                addTelemetry(odo.getPosition(), encoderReady, "Waiting for START", 0.0);
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;

            boolean previousA = gamepad1.a;
            boolean previousX = gamepad1.x;
            boolean moving = false;
            String fault = null;
            String state = "Idle";
            long lastProgressTicks = encoderTicks();
            ElapsedTime noProgressTimer = new ElapsedTime();

            while (opModeIsActive()) {
                odo.update();
                Pose2D pose = odo.getPosition();
                boolean a = gamepad1.a;
                boolean x = gamepad1.x;
                if (a && !previousA) {
                    pose = new Pose2D(DistanceUnit.METER, 0.0, 0.0,
                            AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS));
                    odo.setPosition(pose);
                }

                double heading = pose.getHeading(AngleUnit.RADIANS)
                        + Math.toRadians(HEADING_OFFSET_DEGREES);
                double fieldForward = -gamepad1.left_stick_y;
                double fieldRight = gamepad1.left_stick_x;
                drive(fieldForward * Math.cos(heading) - fieldRight * Math.sin(heading),
                        fieldForward * Math.sin(heading) + fieldRight * Math.cos(heading),
                        gamepad1.right_stick_x);

                long ticks = encoderTicks();
                double angle = encoderReady ? turretAngleDegrees(ticks) : 0.0;
                // Unwrapped angle: never normalize to +/-180, which would hide overtravel.
                // Latch faults until STOP/re-init; X cannot override a limit fault.
                if (encoderReady && Math.abs(angle) >= TURRET_LIMIT_DEGREES) {
                    fault = "180 degree travel limit reached";
                }
                if (updateMagnetZero()) {
                    // The reference changed, not the direction of physical motion.
                    // Restart progress checks so re-zeroing cannot cause a false fault.
                    ticks = encoderTicks();
                    angle = encoderReady ? turretAngleDegrees(ticks) : 0.0;
                    lastProgressTicks = ticks;
                    noProgressTimer.reset();
                }
                if (x && !previousX && encoderReady && fault == null) {
                    moving = angle < TEST_TARGET_DEGREES;
                    state = moving ? "Moving positive to +90 degrees" : "Already at/past +90 degrees";
                    lastProgressTicks = ticks;
                    noProgressTimer.reset();
                }
                if (gamepad1.b) {
                    moving = false;
                    state = "Cancelled";
                }

                if (moving && fault == null) {
                    if (angle >= TEST_TARGET_DEGREES) {
                        moving = false;
                        state = "Reached +90 degrees";
                    } else if (ticks < lastProgressTicks) {
                        fault = "Encoder moved negative: check ENCODER_SIGN / servo direction";
                    } else if (ticks > lastProgressTicks) {
                        lastProgressTicks = ticks;
                        noProgressTimer.reset();
                    } else if (noProgressTimer.seconds() >= ENCODER_NO_PROGRESS_TIMEOUT_SECONDS) {
                        fault = "No encoder progress for 1 second";
                    }
                }
                if (fault != null || !encoderReady) moving = false;
                double turretPower = moving ? TURRET_TEST_POWER : 0.0;
                setTurretPower(turretPower);

                addTelemetry(pose, encoderReady, fault == null ? state : "FAULT: " + fault,
                        turretPower);
                telemetry.update();
                previousA = a;
                previousX = x;
                idle();
            }
        } finally {
            setTurretPower(0.0);
            frontLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backLeft.setPower(0.0);
            backRight.setPower(0.0);
        }
    }

    private long encoderTicks() {
        return turretEncoder == null ? 0L
                : ENCODER_SIGN * ((long) turretEncoder.getCurrentPosition() - turretZeroTicks);
    }

    /** Zero once on entry into the magnet's detection zone, including during INIT. */
    private boolean updateMagnetZero() {
        boolean detected = ZERO_SWITCH_ACTIVE_LOW ? !limitSwitch.getState() : limitSwitch.getState();
        boolean newlyDetected = detected && !magnetDetected;
        if (newlyDetected) {
            turretZeroTicks = turretEncoder.getCurrentPosition();
            magnetZeroEstablished = true;
        }
        // Do not keep resetting while the magnet remains near the sensor: the
        // encoder must continue counting as the turret leaves the detection zone.
        magnetDetected = detected;
        return newlyDetected;
    }

    private double turretAngleDegrees(long ticks) {
        return ticks / ENCODER_TICKS_PER_REVOLUTION / ENCODER_REVOLUTIONS_PER_TURRET_DEGREE;
    }

    private void addTelemetry(Pose2D pose, boolean encoderReady, String state, double power) {
        telemetry.addLine("Left stick: field-relative drive | Right stick X: turn");
        telemetry.addLine("X: move turret to +90 degrees | B: cancel | A: reset odometry X/Y");
        telemetry.addData("Odometry status", odo.getDeviceStatus());
        telemetry.addData("Distance (m) X", "%.3f", pose.getX(DistanceUnit.METER));
        telemetry.addData("Distance (m) Y", "%.3f", pose.getY(DistanceUnit.METER));
        telemetry.addData("Heading (Degrees)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Turret encoder found", turretEncoder != null);
        telemetry.addData("Zero magnet detected", magnetDetected);
        telemetry.addData("Magnet zero established", magnetZeroEstablished);
        telemetry.addData("Zero reference", magnetZeroEstablished ? "Magnet" : "Straight-ahead INIT (provisional)");
        telemetry.addData("Turret encoder ticks (signed, from zero)", encoderTicks());
        if (encoderReady) {
            telemetry.addData("Encoder revolutions", "%.3f", encoderTicks() / ENCODER_TICKS_PER_REVOLUTION);
            telemetry.addData("Turret angle (degrees)", "%.2f", turretAngleDegrees(encoderTicks()));
        } else {
            telemetry.addLine("TURRET DISABLED: ENCODER_SIGN must be +1 or -1.");
        }
        telemetry.addData("Turret state", state);
        telemetry.addData("Turret power (both servos)", "%.2f", power);
        telemetry.addData("Turret limits (degrees)", "+/- %.0f", TURRET_LIMIT_DEGREES);
        telemetry.addData("Future field target (m), INACTIVE", "X %.2f / Y %.2f",
                FIELD_TARGET_X_METERS, FIELD_TARGET_Y_METERS);
    }

    private void setTurretPower(double power) {
        leftTurretServo.setPower(power);
        rightTurretServo.setPower(power);
    }

    private static void configureDriveMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setPower(0.0);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void configurePinpoint() {
        odo.setOffsets(0.4161412697138749, -2.594972445270208, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    private void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }
}
