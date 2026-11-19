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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

/** Field-relative drive, manual turret testing and a repeatable absolute +90 degree move. */
@Configurable
@TeleOp(name = "TurretAimTest")
public class TurretAimTest extends LinearOpMode {
    // Future field aiming target in meters relative to the odometry origin. UNUSED for now.
    private static final double FIELD_TARGET_X_METERS = 1.0;
    private static final double FIELD_TARGET_Y_METERS = 1.0;

    // REV Through Bore Encoder V2 mechanically coupled to the TURRET.
    // Read Expansion Hub encoder port 0 directly, independently of drive motor names.
    // The standard Control Hub -> Expansion Hub RS485 connection makes the
    // Expansion Hub the child module. Never reset its shared motor channel.
    private static final int TURRET_ENCODER_PORT = 0;
    private static final double ENCODER_TICKS_PER_REVOLUTION = 8192.0;
    // Set to -1 if LB (positive turret power) makes the native raw count decrease.
    private static final int ENCODER_SIGN = 1;
    // Original HYPOTHETICAL ratio: 3.75 encoder revolutions for ONE turret degree.
    // This means 30720 counts/degree and 337.5 encoder revolutions for 90 degrees.
    // Replace this value with the measured mechanism ratio; do not confuse it
    // with encoder revolutions per full turret revolution.
    private static final double ENCODER_REVOLUTIONS_PER_TURRET_DEGREE = 3.75;
    private static final String ZERO_SWITCH_NAME = "limitSwitch";
    private static final boolean ZERO_SWITCH_ACTIVE_LOW = true;
    private static final double ZERO_SWITCH_REARM_SECONDS = 0.05;
    private static final double HEADING_OFFSET_DEGREES = 0.0;

    // Live in Panels. Copy tuned values here to preserve them after a restart.
    // Both CR servos drive the SAME turret, so they share one position controller.
    // Auto: fixed bang-bang power outside PDF range, then
    // P * error(deg) - D * velocity(deg/sec) + F * sign(error) inside the range.
    // F is a constant friction feedforward; there is no integral term.
    public static volatile double TURRET_P = 0.02;
    public static volatile double TURRET_D = 0.001;
    public static volatile double TURRET_F = 0.05;
    public static volatile double TURRET_MAX_POWER = 1.0;
    public static volatile double TURRET_BANG_BANG_POWER = 1.0;
    // Switch to PDF when absolute target error is <= this many degrees.
    public static volatile double TURRET_PDF_RANGE_DEGREES = 10.0;
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
    private boolean magnetArmed = true;
    private boolean magnetZeroEstablished;
    private double lastMagnetSeen;
    private int magnetResetCount;

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
            updateMagnetZero(clock.seconds());

            telemetry.setMsTransmissionInterval(100);
            while (opModeInInit()) {
                sampleSensors();
                updateMagnetZero(clock.seconds());
                telemetry.addLine("INIT: keep robot still until odometry calibration finishes.");
                telemetry.addLine("Start turret STRAIGHT; magnet detection corrects zero.");
                addTelemetry(odo.getPosition(), "Waiting for START", 0.0, 0.0);
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;

            boolean previousA = gamepad1.a;
            TurretAimController controller = new TurretAimController(gamepad1.x);
            controller.rebase(turretAngleDegrees(), signedEncoderTicks(), clock.seconds());
            while (opModeIsActive()) {
                sampleSensors();
                double now = clock.seconds();
                if (updateMagnetZero(now)) {
                    controller.rebase(turretAngleDegrees(), signedEncoderTicks(), now);
                }
                Pose2D pose = odo.getPosition();
                boolean a = gamepad1.a;
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

                TurretAimController.Settings settings = new TurretAimController.Settings(
                        TURRET_P, TURRET_D, TURRET_F, TURRET_MAX_POWER, TURRET_MANUAL_POWER,
                        TURRET_BANG_BANG_POWER, TURRET_PDF_RANGE_DEGREES);
                double power = controller.update(turretAngleDegrees(), signedEncoderTicks(), now,
                        gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.x, gamepad1.b, settings);
                if (ENCODER_SIGN != 1 && ENCODER_SIGN != -1) power = 0.0;
                setTurretPower(power);
                addTelemetry(pose, controller.getState(), power, controller.getVelocity());
                telemetry.update();
                previousA = a;
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

    /** Zero on magnet entry, then require a clear interval before the next reset. */
    private boolean updateMagnetZero(double now) {
        magnetDetected = ZERO_SWITCH_ACTIVE_LOW ? !limitSwitch.getState() : limitSwitch.getState();
        boolean reset = false;
        if (magnetDetected) {
            lastMagnetSeen = now;
            if (magnetArmed) {
                zeroEncoderTicks = rawEncoderTicks; // Software offset only; shared motor keeps driving.
                magnetZeroEstablished = true;
                magnetResetCount++;
                magnetArmed = false;
                reset = true;
            }
        } else if (now - lastMagnetSeen >= ZERO_SWITCH_REARM_SECONDS) {
            magnetArmed = true;
        }
        return reset;
    }

    private void addTelemetry(Pose2D pose, String state, double power, double velocity) {
        telemetry.addLine("Left stick: field-relative drive | Right stick X: turn");
        telemetry.addLine("LB: turret + | RB: turret - | Both: stop | X: absolute +90 | B: cancel/clear fault");
        telemetry.addLine("A: reset odometry X/Y | P/D/F editable live in Panels");
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
        telemetry.addData("Encoder tracking", encoderMovementSeen
                ? "Count changes detected" : "No count changes detected since INIT");
        telemetry.addData("Turret LB / RB / X inputs", "%b / %b / %b",
                gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.x);
        telemetry.addLine("Control state does not gate encoder tracking; readings update even while Idle.");
        if (deltaEncoderTicks == 0) {
            telemetry.addLine("If raw ticks stay fixed WHILE MOVING, check Expansion Hub port 0 / encoder cable.");
        }
        telemetry.addData("Zero magnet detected / reset count", "%b / %d", magnetDetected, magnetResetCount);
        telemetry.addData("Zero reference", magnetZeroEstablished ? "Magnet" : "Straight-ahead INIT (provisional)");
        telemetry.addData("Turret state", state);
        telemetry.addData("Turret target / limits (degrees)", "+90 / +/-180 (inward recovery allowed)");
        telemetry.addData("Turret velocity (deg/s)", "%.4f", velocity);
        telemetry.addData("Turret power (both servos)", "%.3f", power);
        telemetry.addData("P / D / F", "%.5f / %.5f / %.5f", TURRET_P, TURRET_D, TURRET_F);
        telemetry.addData("Bang-bang power / PDF range (deg)", "%.3f / %.2f",
                TURRET_BANG_BANG_POWER, TURRET_PDF_RANGE_DEGREES);
        telemetry.addData("Manual / max power", "%.3f / %.3f", TURRET_MANUAL_POWER, TURRET_MAX_POWER);
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
