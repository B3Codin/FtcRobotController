package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
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

import java.util.List;

/** Field-relative drive, continuous field-point aiming and manual turret testing. */
@Configurable
@TeleOp(name = "TurretAimTest")
public class TurretAimTest extends LinearOpMode {
    // Origin is the robot tracking point at INIT. Initial forward is +X, left is +Y.
    // Live in Panels, all distances in meters. Measure pivot offsets from the odometry tracking point.
    // Provisional fit to turret-1795278886850-842194851.csv after reversing the bad X pod.
    // Reconstruction assumes midpoint headings between samples and zero pivot offset.
    // Start at the SAME physical location and orientation as that calibration session.
    public static volatile double FIELD_TARGET_X_METERS = 0.986958491;
    public static volatile double FIELD_TARGET_Y_METERS = -1.312629645;
    // Added to the clockwise encoder target. Includes alignment and reconstruction error.
    public static volatile double TURRET_AIM_TRIM_DEGREES = -4.007486899;
    public static volatile double TURRET_PIVOT_FORWARD_METERS = 0.0;
    public static volatile double TURRET_PIVOT_LEFT_METERS = 0.0;

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
    // Auto: fixed bang-bang power outside PDF range, then
    // P * error(deg) - D * velocity(deg/sec) + F * sign(error) inside the range.
    // F is a constant friction feedforward; there is no integral term.
    public static volatile double TURRET_P = 0.002;
    public static volatile double TURRET_D = 0.0;
    // The robot stalled at 0.058 power near target. Start friction feedforward at
    // the existing 0.20 manual command, then tune on the physical mechanism.
    public static volatile double TURRET_F = 0.20;
    public static volatile double TURRET_MAX_POWER = 1.0;
    public static volatile double TURRET_BANG_BANG_POWER = 1.0;
    // Switch to PDF when absolute target error is <= this many degrees.
    public static volatile double TURRET_PDF_RANGE_DEGREES = 30.0;
    // LB/RB use fixed power, bypassing PDF but obeying the same travel/stall checks.
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
    private double fieldTargetDegrees = Double.NaN;
    private double fieldOriginX;
    private double fieldOriginY;
    private double lastTrackingLogTime = Double.NEGATIVE_INFINITY;

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
                telemetry.addLine("Use the original calibration starting position and chassis orientation.");
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
            TurretAimController controller = new TurretAimController(gamepad1.x, gamepad1.y, gamepad1.b);
            controller.rebase(turretAngleDegrees(), signedEncoderTicks(), clock.seconds());
            while (opModeIsActive()) {
                sampleSensors();
                double now = clock.seconds();
                // Sample once so asynchronous gamepad updates cannot consume an edge
                // between the button check and the previous-button assignment.
                boolean zero = gamepad1.dpad_down;
                boolean track = gamepad1.dpad_up;
                boolean zeroPressed = zero && !previousZero;
                boolean trackPressed = track && !previousTrack;
                if (zeroPressed) {
                    calibrateStraightAhead();
                    controller.cancel(turretAngleDegrees(), signedEncoderTicks(), now);
                }
                if (trackPressed) tracking = !tracking;
                if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.x
                        || gamepad1.y || gamepad1.b || zero) tracking = false;
                Pose2D pose = odo.getPosition();
                boolean a = gamepad1.a;
                if (a && !previousA) {
                    tracking = false;
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
                drive(fieldForward * Math.cos(heading) - fieldRight * Math.sin(heading),
                        fieldForward * Math.sin(heading) + fieldRight * Math.cos(heading),
                        gamepad1.right_stick_x);

                fieldTargetDegrees = TurretAimController.fieldTargetDegrees(
                        pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER), heading,
                        FIELD_TARGET_X_METERS - fieldOriginX, FIELD_TARGET_Y_METERS - fieldOriginY,
                        TURRET_PIVOT_FORWARD_METERS, TURRET_PIVOT_LEFT_METERS,
                        TURRET_AIM_TRIM_DEGREES);
                boolean trackingReady = straightZeroEstablished
                        && odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
                TurretAimController.Settings settings = new TurretAimController.Settings(
                        TURRET_P, TURRET_D, TURRET_F, TURRET_MAX_POWER, TURRET_MANUAL_POWER,
                        TURRET_BANG_BANG_POWER, TURRET_PDF_RANGE_DEGREES);
                double power = controller.update(turretAngleDegrees(), signedEncoderTicks(), now,
                        gamepad1.left_bumper, gamepad1.right_bumper,
                        gamepad1.x, gamepad1.y, gamepad1.b, settings,
                        tracking, trackingReady ? fieldTargetDegrees : Double.NaN);
                if (zero) {
                    controller.cancel(turretAngleDegrees(), signedEncoderTicks(), now);
                    power = 0.0;
                }
                if (ENCODER_SIGN != 1 && ENCODER_SIGN != -1) power = 0.0;
                setTurretPower(power);
                if (now - lastTrackingLogTime >= 0.5) {
                    RobotLog.ii("TurretAimTracking",
                            "enabled=%b state=%s x=%.3f y=%.3f heading=%.2f target=%.2f angle=%.2f power=%.3f odo=%s buttons=up:%b down:%b a:%b x:%b y:%b b:%b lb:%b rb:%b P=%.4f D=%.4f F=%.3f max=%.3f",
                            tracking, controller.getState(), pose.getX(DistanceUnit.METER),
                            pose.getY(DistanceUnit.METER), pose.getHeading(AngleUnit.DEGREES),
                            fieldTargetDegrees, turretAngleDegrees(), power, odo.getDeviceStatus(),
                            track, zero, gamepad1.a, gamepad1.x,
                            gamepad1.y, gamepad1.b, gamepad1.left_bumper, gamepad1.right_bumper,
                            settings.p, settings.d, settings.f, settings.maxPower);
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
    }

    private void addTelemetry(Pose2D pose, String state, double power, double velocity, double target) {
        telemetry.addLine("Left stick: field-relative drive | Right stick X: turn");
        telemetry.addLine("LB: turret + | RB: turret - | Both: stop/clear fault");
        telemetry.addLine("Absolute targets: X +90 | Y 0 | B -90 | Release/re-press any command to retry a fault");
        telemetry.addLine("D-pad Down: set straight-ahead zero | D-pad Up: toggle field tracking");
        telemetry.addLine("A: cancel tracking and rebase robot/goal X/Y | P/D/F in Panels");
        telemetry.addLine("Provisional corrected-data fit; use original starting pose.");
        telemetry.addData("Aim trim (clockwise degrees)", "%.2f", TURRET_AIM_TRIM_DEGREES);
        telemetry.addData("Tracking requested", tracking);
        telemetry.addData("Turret state", state);
        telemetry.addData("Turret power (both servos)", "%.3f", power);
        telemetry.addData("Odometry status", odo.getDeviceStatus());
        telemetry.addData("Field target angle (clockwise degrees)", "%.2f", fieldTargetDegrees);
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
        telemetry.addData("P / D / F", "%.5f / %.5f / %.5f", TURRET_P, TURRET_D, TURRET_F);
        telemetry.addData("Bang-bang power / PDF range (deg)", "%.3f / %.2f",
                TURRET_BANG_BANG_POWER, TURRET_PDF_RANGE_DEGREES);
        telemetry.addData("Manual / max power", "%.3f / %.3f", TURRET_MANUAL_POWER, TURRET_MAX_POWER);
        telemetry.addData("Field point from odometry origin (m)", "X %.2f / Y %.2f",
                FIELD_TARGET_X_METERS - fieldOriginX, FIELD_TARGET_Y_METERS - fieldOriginY);
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
        // Forward chassis travel was physically verified to require X FORWARD.
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
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
