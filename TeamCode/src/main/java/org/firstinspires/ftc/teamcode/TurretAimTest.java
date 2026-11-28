package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

/** Field-relative drive, continuous field-point aiming and manual turret testing. */
@Configurable
@TeleOp(name = "TurretAimTest")
public class TurretAimTest extends LinearOpMode {
    // Share JackbotsTeleOp's field origin, starting pose, turret offset and goal selection.
    // Field targeting below uses its TurretTarget calculation directly.

    // REV Through Bore Encoder V2 mechanically coupled to the TURRET.
    // Read Expansion Hub encoder port 0 directly, independently of drive motor names.
    // The standard Control Hub -> Expansion Hub RS485 connection makes the
    // Expansion Hub the child module. Never reset its shared motor channel.
    private static final int TURRET_ENCODER_PORT = 0;
    private static final double ENCODER_TICKS_PER_REVOLUTION = 8192.0;
    // Set to -1 if LB (positive turret power) makes the native raw count decrease.
    private static final int ENCODER_SIGN = 1;
    // 2048 quadrature cycles = 8192 Hub counts per encoder revolution.
    // The 24-tooth encoder gear turns 90/24 = 3.75 times per TURRET revolution.
    // Therefore 8192 * 3.75 / 360 = 85.3333 counts per turret degree.
    private static final double ENCODER_GEAR_TEETH = 24.0;
    private static final double TURRET_GEAR_TEETH = 90.0;
    private static final double ENCODER_REVOLUTIONS_PER_TURRET_DEGREE =
            (TURRET_GEAR_TEETH / ENCODER_GEAR_TEETH) / 360.0;
    private static final String ZERO_SWITCH_NAME = "limitSwitch";
    private static final boolean ZERO_SWITCH_ACTIVE_LOW = true;
    private static final double HEADING_OFFSET_DEGREES = 0.0;

    // Live in Panels. Copy tuned values here to preserve them after a restart.
    // Both CR servos drive the SAME turret, so they share one position controller.
    // Defaults match JackbotsTeleOp. PIDF covers the full +/-180 degree travel range.
    public static volatile double TURRET_P = 0.005;
    public static volatile double TURRET_I = 0.0;
    public static volatile double TURRET_D = 0.0;
    public static volatile double TURRET_F = 0.08;
    public static volatile double TURRET_MIN_POWER = 0.1;
    public static volatile double TURRET_MAX_POWER = 1.0;
    public static volatile double TURRET_TOLERANCE_DEGREES = 2.0;
    // LB/RB use fixed power, bypassing PIDF but obeying the same travel/stall checks.
    public static volatile double TURRET_MANUAL_POWER = 0.2;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DigitalChannel limitSwitch;
    private CRServo leftTurretServo, rightTurretServo;
    private GoBildaPinpointDriver odo;
    private List<LynxModule> hubs;
    private LynxModule turretEncoderHub;
    private boolean encoderSampled;
    private boolean encoderMovementSeen;
    private int rawEncoderTicks;
    private int zeroEncoderTicks;
    private long deltaEncoderTicks;
    private boolean magnetDetected;
    private boolean straightZeroEstablished;
    private boolean tracking;
    private final FieldHeadingSelection fieldHeading = new FieldHeadingSelection();
    private double fieldTargetDegrees = Double.NaN;
    private double fieldOriginX;
    private double fieldOriginY;
    private double lastTrackingLogTime = Double.NEGATIVE_INFINITY;
    private final ElapsedTime pinpointStartupClock = new ElapsedTime();
    private final JackbotsTeleOp.StartingPose startingPose = new JackbotsTeleOp.StartingPose();

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontLeftName.get());
        frontRight = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontRightName.get());
        backLeft = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backLeftName.get());
        backRight = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backRightName.get());
        leftTurretServo = hardwareMap.get(CRServo.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(CRServo.class, "rightTurretServo");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());

        try {
            setTurretPower(0.0);
            configureDriveMotor(frontLeft, Constants.drivetrainConfig.frontLeftDirection.get());
            configureDriveMotor(backLeft, Constants.drivetrainConfig.backLeftDirection.get());
            configureDriveMotor(frontRight, Constants.drivetrainConfig.frontRightDirection.get());
            configureDriveMotor(backRight, Constants.drivetrainConfig.backRightDirection.get());
            configurePinpoint();
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) {
                if (!hub.isParent()) {
                    if (turretEncoderHub != null) {
                        throw new IllegalStateException("Multiple Expansion Hubs: select turret encoder Hub explicitly.");
                    }
                    turretEncoderHub = hub;
                }
            }
            if (turretEncoderHub == null) {
                throw new IllegalStateException("No child Expansion Hub found. Check RS485 connection and robot configuration.");
            }
            limitSwitch = hardwareMap.get(DigitalChannel.class, ZERO_SWITCH_NAME);
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
            ElapsedTime clock = new ElapsedTime();
            sampleSensors();
            zeroEncoderTicks = rawEncoderTicks;
            boolean previousZero = gamepad1.dpad_down;

            telemetry.setMsTransmissionInterval(100);
            while (opModeInInit()) {
                sampleSensors();
                if (gamepad1.dpad_down && !previousZero) calibrateStraightAhead();
                previousZero = gamepad1.dpad_down;
                telemetry.addLine("INIT: keep robot still until odometry calibration finishes.");
                telemetry.addLine("Use JackbotsTeleOp's starting pose: X 198 mm, Y -217.4 mm, heading 0 degrees.");
                telemetry.addLine("Align physical straight-ahead mark, then press D-pad Down to set zero.");
                addTelemetry(odo.getPosition(), "Waiting for START", 0.0, 0.0, 90.0);
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;

            boolean previousA = gamepad1.a;
            boolean previousTrack = gamepad1.dpad_up;
            previousZero = gamepad1.dpad_down;
            fieldHeading.onStart(gamepad1.x, gamepad1.y, gamepad1.b);
            TurretAimController controller = new TurretAimController(gamepad1.x, gamepad1.y, gamepad1.b);
            controller.rebase(turretAngleDegrees(), signedEncoderTicks(), clock.seconds());
            while (opModeIsActive()) {
                sampleSensors();
                double now = clock.seconds();
                // Sample once so asynchronous gamepad updates cannot consume an edge
                // between the button check and the previous-button assignment.
                boolean zero = gamepad1.dpad_down;
                boolean track = gamepad1.dpad_up;
                boolean x = gamepad1.x, y = gamepad1.y, b = gamepad1.b;
                boolean leftBumper = gamepad1.left_bumper, rightBumper = gamepad1.right_bumper;
                boolean zeroPressed = zero && !previousZero;
                boolean trackPressed = track && !previousTrack;
                if (zeroPressed) {
                    calibrateStraightAhead();
                    controller.cancel(turretAngleDegrees(), signedEncoderTicks(), now);
                }
                if (trackPressed) {
                    tracking = !tracking;
                    fieldHeading.clear();
                    controller.cancel(turretAngleDegrees(), signedEncoderTicks(), now);
                }
                if (fieldHeading.update(x, y, b)) {
                    tracking = false;
                    // A fresh preset press changes direction or explicitly retries a fault.
                    controller.cancel(turretAngleDegrees(), signedEncoderTicks(), now);
                }
                if (leftBumper || rightBumper || zero) {
                    tracking = false;
                    fieldHeading.clear();
                }
                Pose2D pose = odo.getPosition();
                boolean a = gamepad1.a;
                if (startingPose.isInitialized() && a && !previousA) {
                    tracking = false;
                    fieldHeading.clear();
                    controller.cancel(turretAngleDegrees(), signedEncoderTicks(), now);
                    // Translate the stored goal with the origin so it stays at the same field point.
                    fieldOriginX += pose.getX(DistanceUnit.METER);
                    fieldOriginY += pose.getY(DistanceUnit.METER);
                    pose = new Pose2D(DistanceUnit.METER, 0.0, 0.0,
                            AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS));
                    odo.setPosition(pose);
                }

                double heading = pose.getHeading(AngleUnit.RADIANS)
                        + Math.toRadians(HEADING_OFFSET_DEGREES);
                double fieldForward = -gamepad1.left_stick_y;
                double fieldRight = gamepad1.left_stick_x;
                if (startingPose.isInitialized()) {
                    drive(fieldForward * Math.cos(heading) - fieldRight * Math.sin(heading),
                            fieldForward * Math.sin(heading) + fieldRight * Math.cos(heading),
                            gamepad1.right_stick_x);
                } else {
                    drive(0.0, 0.0, 0.0);
                }

                fieldTargetDegrees = fieldHeading.isSelected()
                        ? fieldHeading.robotTargetDegrees(pose.getHeading(AngleUnit.DEGREES))
                        : fieldTarget(pose).idealTheta;
                boolean trackingReady = straightZeroEstablished && startingPose.isInitialized()
                        && odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
                TurretAimController.Settings settings = new TurretAimController.Settings(
                        TURRET_P, TURRET_I, TURRET_D, TURRET_F, TURRET_MIN_POWER, TURRET_MAX_POWER,
                        TURRET_MANUAL_POWER, TURRET_MAX_POWER, 360.0, TURRET_TOLERANCE_DEGREES);
                double power = controller.update(turretAngleDegrees(), signedEncoderTicks(), now,
                        leftBumper, rightBumper, false, false, false, settings,
                        tracking || fieldHeading.isSelected(), trackingReady ? fieldTargetDegrees : Double.NaN);
                if (zero) {
                    controller.cancel(turretAngleDegrees(), signedEncoderTicks(), now);
                    power = 0.0;
                }
                if (ENCODER_SIGN != 1 && ENCODER_SIGN != -1) power = 0.0;
                setTurretPower(power);
                if (now - lastTrackingLogTime >= 0.5) {
                    RobotLog.ii("TurretAimTracking",
                            "enabled=%b state=%s x=%.3f y=%.3f heading=%.2f target=%.2f angle=%.2f power=%.3f odo=%s buttons=up:%b down:%b a:%b x:%b y:%b b:%b lb:%b rb:%b P=%.4f I=%.4f D=%.4f F=%.3f min=%.3f max=%.3f tolerance=%.2f",
                            tracking || fieldHeading.isSelected(), controller.getState(), pose.getX(DistanceUnit.METER),
                            pose.getY(DistanceUnit.METER), pose.getHeading(AngleUnit.DEGREES),
                            fieldTargetDegrees, turretAngleDegrees(), power, odo.getDeviceStatus(),
                            track, zero, a, x, y, b, leftBumper, rightBumper,
                            settings.p, settings.i, settings.d, settings.f,
                            settings.minPower, settings.maxPower, settings.angleToleranceDegrees);
                    lastTrackingLogTime = now;
                }
                addTelemetry(pose, controller.getState(), power, controller.getVelocity(), controller.getTargetDegrees());
                telemetry.update();
                previousA = a;
                previousTrack = track;
                previousZero = zero;
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

    private void sampleSensors() {
        // Refresh even if another OpMode left a Hub using MANUAL bulk caching.
        for (LynxModule hub : hubs) hub.clearBulkCache();
        odo.update();
        if (startingPose.shouldApply(odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY,
                pinpointStartupClock.seconds())) {
            odo.setPosition(new Pose2D(DistanceUnit.MM, JackbotsTeleOp.START_X_MM,
                    JackbotsTeleOp.START_Y_MM, AngleUnit.DEGREES, 0.0));
            odo.update();
            Pose2D readback = odo.getPosition();
            startingPose.confirm(odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY,
                    readback.getX(DistanceUnit.MM), readback.getY(DistanceUnit.MM),
                    readback.getHeading(AngleUnit.DEGREES));
        }
        magnetDetected = ZERO_SWITCH_ACTIVE_LOW ? !limitSwitch.getState() : limitSwitch.getState();
        LynxModule.BulkData encoderData = turretEncoderHub.getBulkData();
        if (encoderData.isFake()) {
            throw new IllegalStateException("Expansion Hub encoder read failed; stopping instead of reporting zero.");
        }
        int nextTicks = encoderData.getMotorCurrentPosition(TURRET_ENCODER_PORT);
        deltaEncoderTicks = encoderSampled ? (long) nextTicks - rawEncoderTicks : 0L;
        encoderMovementSeen |= deltaEncoderTicks != 0;
        encoderSampled = true;
        rawEncoderTicks = nextTicks;
    }

    private long signedEncoderTicks() {
        return ENCODER_SIGN * ((long) rawEncoderTicks - zeroEncoderTicks);
    }

    private double turretAngleDegrees() {
        return signedEncoderTicks() / ENCODER_TICKS_PER_REVOLUTION
                / ENCODER_REVOLUTIONS_PER_TURRET_DEGREE;
    }

    /** A physical mark defines zero because the magnetic switch covers several degrees. */
    private void calibrateStraightAhead() {
        zeroEncoderTicks = rawEncoderTicks;
        straightZeroEstablished = true;
        tracking = false;
        fieldHeading.clear();
    }

    /** Preset field bearings are clockwise-positive, relative to the initial forward direction. */
    static final class FieldHeadingSelection {
        private boolean previousX, previousY, previousB;
        private double degrees = Double.NaN;

        void onStart(boolean x, boolean y, boolean b) {
            previousX = x;
            previousY = y;
            previousB = b;
        }

        boolean update(boolean x, boolean y, boolean b) {
            boolean xPressed = x && !previousX;
            boolean yPressed = y && !previousY;
            boolean bPressed = b && !previousB;
            onStart(x, y, b);
            if (!xPressed && !yPressed && !bPressed) return false;
            degrees = yPressed ? 0.0 : xPressed ? 90.0 : -90.0;
            return true;
        }

        void clear() {
            degrees = Double.NaN;
        }

        boolean isSelected() {
            return Double.isFinite(degrees);
        }

        double robotTargetDegrees(double headingDegrees) {
            // Pinpoint heading is CCW-positive; the turret encoder is CW-positive.
            // Add chassis heading to hold a fixed CW field bearing, then wrap to travel limits.
            double target = degrees + headingDegrees;
            return ((target + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        }
    }

    private JackbotsTeleOp.TurretTarget fieldTarget(Pose2D pose) {
        return new JackbotsTeleOp.TurretTarget(
                pose.getX(DistanceUnit.MM) + fieldOriginX * 1000.0,
                pose.getY(DistanceUnit.MM) + fieldOriginY * 1000.0,
                pose.getHeading(AngleUnit.RADIANS));
    }

    private void addTelemetry(Pose2D pose, String state, double power, double velocity, double target) {
        telemetry.addLine("Left stick: field-relative drive | Right stick X: turn");
        telemetry.addLine("LB: turret + | RB: turret - | Both: stop/clear fault");
        telemetry.addLine("Hold field direction: X +90 | Y 0 | B -90 (clockwise from initial forward)");
        telemetry.addLine("Release/re-press a preset to retry a fault. LB/RB cancel the field hold.");
        telemetry.addLine("D-pad Down: set straight-ahead zero | D-pad Up: toggle field tracking");
        telemetry.addLine("A: cancel tracking and reset local X/Y | P/I/D/F in Panels");
        telemetry.addLine("Field goals, turret offset and starting pose match JackbotsTeleOp.");
        telemetry.addData("Starting field pose applied", startingPose.isInitialized());
        telemetry.addData("Tracking requested", tracking || fieldHeading.isSelected());
        telemetry.addData("Aim mode", fieldHeading.isSelected() ? "Field direction" : tracking ? "Field goal" : "Manual / idle");
        telemetry.addData("Selected field direction (CW degrees)", fieldHeading.degrees);
        telemetry.addData("Turret state", state);
        telemetry.addData("Turret power (both servos)", "%.3f", power);
        telemetry.addData("Odometry status", odo.getDeviceStatus());
        telemetry.addData("Required turret angle (robot-relative CW degrees)", "%.2f", fieldTargetDegrees);
        telemetry.addData("Field aiming error (degrees)", "%.2f", fieldTargetDegrees - turretAngleDegrees());
        telemetry.addData("Distance (m) X", "%.3f", pose.getX(DistanceUnit.METER));
        telemetry.addData("Distance (m) Y", "%.3f", pose.getY(DistanceUnit.METER));
        telemetry.addData("Heading (Degrees)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Turret encoder Hub", hardwareMap.getNamesOf(turretEncoderHub));
        telemetry.addData("Turret encoder port", TURRET_ENCODER_PORT);
        telemetry.addData("Native raw encoder ticks (unzeroed)", rawEncoderTicks);
        telemetry.addData("Raw encoder tick change this loop", deltaEncoderTicks);
        telemetry.addData("Turret encoder ticks (signed, from zero)", signedEncoderTicks());
        telemetry.addData("Encoder revolutions", "%.6f", signedEncoderTicks() / ENCODER_TICKS_PER_REVOLUTION);
        telemetry.addData("Turret angle (degrees)", "%.6f", turretAngleDegrees());
        telemetry.addData("Counts per turret degree", "%.6f",
                ENCODER_TICKS_PER_REVOLUTION * ENCODER_REVOLUTIONS_PER_TURRET_DEGREE);
        telemetry.addData("Expected counts at +90 degrees", "%.0f",
                90.0 * ENCODER_TICKS_PER_REVOLUTION * ENCODER_REVOLUTIONS_PER_TURRET_DEGREE);
        telemetry.addData("Encoder tracking", encoderMovementSeen
                ? "Count changes detected" : "No count changes detected since INIT");
        telemetry.addData("Turret LB / RB / X / Y / B inputs", "%b / %b / %b / %b / %b",
                gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.x, gamepad1.y, gamepad1.b);
        telemetry.addLine("Control state does not gate encoder tracking; readings update even while Idle.");
        if (deltaEncoderTicks == 0) {
            telemetry.addLine("If raw ticks stay fixed WHILE MOVING, check Expansion Hub port 0 / encoder cable.");
        }
        telemetry.addData("Magnet zone detected", magnetDetected);
        telemetry.addData("Zero reference", straightZeroEstablished
                ? "Physical straight-ahead mark" : "Uncalibrated: align mark and press D-pad Down");
        telemetry.addData("Turret target / limits (degrees)", "%.0f / +/-180 (inward recovery allowed)", target);
        telemetry.addData("Turret velocity (deg/s)", "%.4f", velocity);
        telemetry.addData("P / I / D / F", "%.5f / %.5f / %.5f / %.5f",
                TURRET_P, TURRET_I, TURRET_D, TURRET_F);
        telemetry.addData("PIDF range / tolerance (deg)", "360 / %.2f", TURRET_TOLERANCE_DEGREES);
        telemetry.addData("Manual / min / max power", "%.3f / %.3f / %.3f",
                TURRET_MANUAL_POWER, TURRET_MIN_POWER, TURRET_MAX_POWER);
        JackbotsTeleOp.TurretTarget fieldTarget = fieldTarget(pose);
        telemetry.addData("Selected goal", fieldTarget.useGoalTwo ? 2 : 1);
        telemetry.addData("Field robot X / Y (mm)", "%.1f / %.1f",
                pose.getX(DistanceUnit.MM) + fieldOriginX * 1000.0,
                pose.getY(DistanceUnit.MM) + fieldOriginY * 1000.0);
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
        PinpointConfig config = Constants.localizerConfig;
        odo.setOffsets(config.xPodOffset.get(), config.yPodOffset.get(), config.offsetUnits.get());
        if (config.ticksPerUnit.get().isPresent()) {
            odo.setEncoderResolution(config.ticksPerUnit.get().getAsDouble(),
                    config.encoderResolutionUnit.get());
        } else {
            odo.setEncoderResolution(config.podType.get());
        }
        odo.setEncoderDirections(config.xPodDirection.get(), config.yPodDirection.get());
        odo.resetPosAndIMU();
        pinpointStartupClock.reset();
        odo.update();
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
