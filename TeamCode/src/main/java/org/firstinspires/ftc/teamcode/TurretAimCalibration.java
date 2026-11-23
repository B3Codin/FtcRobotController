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
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Locale;

/** Collects manually aligned goal observations for fitting turret aiming geometry. */
@Configurable
@TeleOp(name = "TurretAimCalibration", group = "Sensor")
public class TurretAimCalibration extends LinearOpMode {
    // Same coordinate convention as TurretAimTest: INIT origin, +X forward, +Y left.
    // Enable known coordinates in Panels after entering measurements, before START.
    public static volatile boolean GOAL_COORDINATES_KNOWN = false;
    public static volatile double GOAL_X_METERS = 0.0;
    public static volatile double GOAL_Y_METERS = 0.0;
    public static volatile double TURRET_MANUAL_POWER = 0.12;
    public static volatile double DRIVE_POWER = 0.35;

    private static final int ENCODER_PORT = 0;
    private static final int ENCODER_SIGN = 1;
    private static final double TICKS_PER_DEGREE = 8192.0 * (90.0 / 24.0) / 360.0;
    private static final double SETTLE_SECONDS = 0.4;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private CRServo leftTurret, rightTurret;
    private DigitalChannel magnet;
    private GoBildaPinpointDriver odo;
    private List<LynxModule> hubs;
    private LynxModule encoderHub;
    private Pose2D pose;
    private int rawTicks;
    private int zeroTicks;
    private boolean zeroEstablished;
    private boolean magnetDetected;
    private int samples;
    private int approachDirection;
    private double goalX, goalY;
    private double stableSince;
    private double stableX, stableY, stableHeading, stableAngle;
    private String message = "Align straight-ahead mark and press D-pad Down.";
    private File csvFile;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "FLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FRight");
        backLeft = hardwareMap.get(DcMotor.class, "BLeft");
        backRight = hardwareMap.get(DcMotor.class, "BRight");
        leftTurret = hardwareMap.get(CRServo.class, "leftTurretServo");
        rightTurret = hardwareMap.get(CRServo.class, "rightTurretServo");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        magnet = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        try {
            stopMotion();
            configureMotor(frontLeft, DcMotor.Direction.FORWARD);
            configureMotor(backLeft, DcMotor.Direction.FORWARD);
            configureMotor(frontRight, DcMotor.Direction.REVERSE);
            configureMotor(backRight, DcMotor.Direction.REVERSE);
            magnet.setMode(DigitalChannel.Mode.INPUT);
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) {
                if (!hub.isParent()) {
                    if (encoderHub != null) {
                        throw new IllegalStateException("Multiple Expansion Hubs; select turret encoder Hub explicitly.");
                    }
                    encoderHub = hub;
                }
            }
            if (encoderHub == null) throw new IllegalStateException("Turret encoder Expansion Hub missing.");

            // Match the measured odometry configuration used by TurretAimTest.
            odo.setOffsets(0.4161412697138749, -2.594972445270208, DistanceUnit.INCH);
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            // Forward chassis travel was physically verified to require X FORWARD.
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
            odo.resetPosAndIMU();
            ElapsedTime clock = new ElapsedTime();
            readSensors();
            zeroTicks = rawTicks;
            boolean previousZero = gamepad1.dpad_down;
            telemetry.setMsTransmissionInterval(100);

            while (opModeInInit()) {
                readSensors();
                if (gamepad1.dpad_down && !previousZero) setStraightZero();
                previousZero = gamepad1.dpad_down;
                readGoalSettings();
                telemetry.addLine("INIT: keep chassis still for odometry calibration.");
                telemetry.addLine("Enter goal X/Y and enable GOAL_COORDINATES_KNOWN in Panels, or leave unknown.");
                showTelemetry("Waiting for START", false);
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;

            // Freeze the goal for this session so every observation describes the same point.
            readGoalSettings();
            File directory = new File(AppUtil.ROBOT_DATA_DIR, "turret-calibration");
            if (!directory.isDirectory() && !directory.mkdirs()) {
                throw new IOException("Cannot create CSV directory " + directory);
            }
            csvFile = File.createTempFile("turret-" + System.currentTimeMillis() + "-", ".csv", directory);
            try (FileOutputStream stream = new FileOutputStream(csvFile);
                 BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(stream, StandardCharsets.UTF_8))) {
                writer.write("sample,elapsed_s,unix_time_ms,robot_x_m,robot_y_m,heading_ccw_deg,"
                        + "raw_encoder_ticks,zero_encoder_ticks,signed_encoder_ticks,turret_cw_deg,"
                        + "magnet_detected,last_turret_power_sign,goal_x_m,goal_y_m,"
                        + "encoder_sign,ticks_per_turret_degree,odometry_status,"
                        + "odometry_x_direction,odometry_y_direction\n");
                writer.flush();
                stream.getFD().sync();
                boolean previousX = gamepad1.x;
                previousZero = gamepad1.dpad_down;
                TurretAimController controller = new TurretAimController(false, false, false);
                controller.rebase(turretAngle(), signedTicks(), clock.seconds());
                updateStability(clock.seconds(), true);

                while (opModeIsActive()) {
                    readSensors();
                    double now = clock.seconds();
                    boolean neutral = controlsNeutral();
                    boolean stopping = gamepad1.b || gamepad1.x || gamepad1.dpad_down;
                    boolean zeroPressed = gamepad1.dpad_down && !previousZero;
                    if (zeroPressed) {
                        if (samples == 0 && neutral) {
                            setStraightZero();
                            controller.cancel(turretAngle(), signedTicks(), now);
                        } else {
                            message = "Zero requires released controls and zero saved samples.";
                        }
                    }

                    double drivePower = boundedPower(DRIVE_POWER, 1.0);
                    double heading = pose.getHeading(AngleUnit.RADIANS);
                    if (stopping || !Double.isFinite(heading) || driveControlsNeutral()) {
                        drive(0, 0, 0);
                    } else {
                        double forward = -gamepad1.left_stick_y * drivePower;
                        double right = gamepad1.left_stick_x * drivePower;
                        drive(forward * Math.cos(heading) - right * Math.sin(heading),
                                forward * Math.sin(heading) + right * Math.cos(heading),
                                gamepad1.right_stick_x * drivePower);
                    }
                    double manualPower = boundedPower(TURRET_MANUAL_POWER, 0.3);
                    TurretAimController.Settings settings = new TurretAimController.Settings(
                            0, 0, 0, 0.3, manualPower, 0.3, 30);
                    // Reuse the existing manual travel, stall and encoder-direction checks.
                    double turretPower = controller.update(turretAngle(), signedTicks(), now,
                            stopping || gamepad1.left_bumper, stopping || gamepad1.right_bumper,
                            false, false, false, settings);
                    setTurretPower(turretPower);
                    if (turretPower != 0) approachDirection = (int) Math.signum(turretPower);
                    boolean ready = updateStability(now, !neutral || zeroPressed)
                            && zeroEstablished && validPose();

                    if (gamepad1.x && !previousX) {
                        if (ready) {
                            writeSample(writer, stream, now);
                            message = "Saved sample " + samples;
                            gamepad1.rumble(100);
                        } else {
                            message = "Sample skipped. Set zero, release controls, wait for READY, then press X again.";
                        }
                    }
                    showTelemetry(controller.getState(), ready);
                    previousX = gamepad1.x;
                    previousZero = gamepad1.dpad_down;
                    idle();
                }
            }
        } catch (IOException failure) {
            throw new IllegalStateException("Calibration CSV failed; confirmed samples " + samples
                    + ", file " + csvFile, failure);
        } finally {
            stopMotion();
        }
    }

    private void readSensors() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        odo.update();
        pose = odo.getPosition();
        LynxModule.BulkData data = encoderHub.getBulkData();
        if (data.isFake()) throw new IllegalStateException("Turret encoder read failed.");
        rawTicks = data.getMotorCurrentPosition(ENCODER_PORT);
        magnetDetected = !magnet.getState();
    }

    private void setStraightZero() {
        zeroTicks = rawTicks;
        zeroEstablished = true;
        approachDirection = 0;
        message = "Straight-ahead zero recorded.";
    }

    private long signedTicks() {
        return ENCODER_SIGN * ((long) rawTicks - zeroTicks);
    }

    private double turretAngle() {
        return signedTicks() / TICKS_PER_DEGREE;
    }

    private boolean validPose() {
        return odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                && Double.isFinite(pose.getX(DistanceUnit.METER))
                && Double.isFinite(pose.getY(DistanceUnit.METER))
                && Double.isFinite(pose.getHeading(AngleUnit.RADIANS));
    }

    private boolean controlsNeutral() {
        return driveControlsNeutral() && !gamepad1.left_bumper && !gamepad1.right_bumper;
    }

    private boolean driveControlsNeutral() {
        return Math.abs(gamepad1.left_stick_x) < 0.02 && Math.abs(gamepad1.left_stick_y) < 0.02
                && Math.abs(gamepad1.right_stick_x) < 0.02;
    }

    private void readGoalSettings() {
        goalX = GOAL_COORDINATES_KNOWN ? GOAL_X_METERS : Double.NaN;
        goalY = GOAL_COORDINATES_KNOWN ? GOAL_Y_METERS : Double.NaN;
    }

    /** Require the pose to remain within 3 mm / 0.3 degrees of an anchor for 0.4 s. */
    private boolean updateStability(double now, boolean moving) {
        double x = pose.getX(DistanceUnit.METER);
        double y = pose.getY(DistanceUnit.METER);
        double heading = pose.getHeading(AngleUnit.RADIANS);
        double headingChange = Math.atan2(Math.sin(heading - stableHeading), Math.cos(heading - stableHeading));
        if (moving || !validPose() || !Double.isFinite(stableX) || !Double.isFinite(stableY)
                || !Double.isFinite(stableHeading) || Math.hypot(x - stableX, y - stableY) > 0.003
                || Math.abs(headingChange) > Math.toRadians(0.3)
                || Math.abs(turretAngle() - stableAngle) > 0.3) {
            stableSince = now;
            stableX = x;
            stableY = y;
            stableHeading = heading;
            stableAngle = turretAngle();
        }
        return now - stableSince >= SETTLE_SECONDS;
    }

    private void writeSample(BufferedWriter writer, FileOutputStream stream, double now) throws IOException {
        writer.write(String.format(Locale.US,
                "%d,%.6f,%d,%.6f,%.6f,%.6f,%d,%d,%d,%.6f,%b,%d,%.6f,%.6f,%d,%.9f,%s,FORWARD,FORWARD%n",
                samples + 1, now, System.currentTimeMillis(), pose.getX(DistanceUnit.METER),
                pose.getY(DistanceUnit.METER), pose.getHeading(AngleUnit.DEGREES), rawTicks,
                zeroTicks, signedTicks(), turretAngle(), magnetDetected, approachDirection,
                goalX, goalY, ENCODER_SIGN, TICKS_PER_DEGREE, odo.getDeviceStatus()));
        writer.flush();
        stream.getFD().sync();
        samples++;
    }

    private void showTelemetry(String controllerState, boolean ready) {
        telemetry.addLine("Left stick drives field-relative | Right stick X turns chassis");
        telemetry.addLine("Hold LB/RB to turn turret slowly | Release to stop | B stops all motion");
        telemetry.addLine("X saves one aligned sample | D-pad Down sets straight zero before first sample");
        telemetry.addData("Capture", ready ? "READY" : "Waiting for calibration / stationary pose");
        telemetry.addData("Last action", message);
        telemetry.addData("Saved samples", samples);
        telemetry.addData("CSV file", csvFile == null ? "Created at START" : csvFile.getAbsolutePath());
        telemetry.addData("Goal X/Y (m)", "%.3f / %.3f", goalX, goalY);
        telemetry.addData("Robot X/Y (m)", "%.4f / %.4f", pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER));
        telemetry.addData("Heading CCW (deg)", "%.3f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Raw / zero encoder ticks", "%d / %d", rawTicks, zeroTicks);
        telemetry.addData("Turret CW (deg)", "%.3f", turretAngle());
        telemetry.addData("Straight zero", zeroEstablished ? "Recorded" : "Align mark and press D-pad Down");
        telemetry.addData("Magnet zone", magnetDetected);
        telemetry.addData("Odometry", odo.getDeviceStatus());
        telemetry.addData("Odometry X / Y direction", "FORWARD / FORWARD");
        telemetry.addData("Turret control", controllerState);
        telemetry.update();
    }

    private static double boundedPower(double value, double maximum) {
        return Double.isFinite(value) ? Math.max(0, Math.min(maximum, value)) : 0;
    }

    private static void configureMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setPower(0);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setTurretPower(double power) {
        leftTurret.setPower(power);
        rightTurret.setPower(power);
    }

    private void stopMotion() {
        setTurretPower(0);
        drive(0, 0, 0);
    }

    private void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;
        double scale = Math.max(1, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / scale);
        frontRight.setPower(fr / scale);
        backLeft.setPower(bl / scale);
        backRight.setPower(br / scale);
    }
}
