package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.List;
@TeleOp (name = "AprilTag")
public class AprilTag extends LinearOpMode {
    // Adjust so Pinpoint heading + this offset matches the Limelight field map's yaw.
    private static final double FIELD_HEADING_OFFSET_DEGREES = 0.0;
    private static final int APRILTAG_PIPELINE = 0;
    private static final double TARGET_DISTANCE_METERS = 3.0;
    private static final double DISTANCE_TOLERANCE_METERS = 0.05;
    private static final double TX_TOLERANCE_DEGREES = 1.0;
    // Allow distance correction while steering; the final tx tolerance stays at 1 degree.
    private static final double MAX_DRIVE_TX_DEGREES = 10.0;
    private static final double DRIVE_GAIN = 0.25;
    private static final double TURN_GAIN = 0.05;
    // Power per degree/second of clockwise rotation; slows a turn before it overshoots.
    private static final double TURN_DAMPING_GAIN = 0.002;
    private static final double TURN_SLOWDOWN_DEGREES = 10.0;
    private static final double MAX_DRIVE_POWER = 0.6;
    private static final double MAX_TURN_POWER = 0.4;
    private static final double MIN_DRIVE_POWER = 0.2;
    private static final double MIN_TURN_POWER = 0.2;
    private static final long MAX_RESULT_AGE_MS = 150;
    private static final long SETTLE_TIME_NS = 250_000_000L;
    private static final long ALIGNMENT_TIMEOUT_NS = 15_000_000_000L;

    private Limelight3A limelight;
    private GoBildaPinpointDriver odo;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private boolean aligning;
    private boolean aWasPressed;
    private int alignmentTagId;
    private long alignmentStartedNs;
    private long inToleranceSinceNs;
    private double lastCameraTimestamp = Double.NaN;
    private long lastNewFrameNs;
    private double clockwiseTurnRateDegrees;
    private String alignmentStatus = "Idle: press A to align; A or B cancels.";

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        initializeDrive();
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        try {
            limelight.start();
            telemetry.addLine("Ready. Pinpoint heading must match the Limelight field map.");
            telemetry.addLine("A: align to tx 0 +/- 1 deg and tag distance 3.00 +/- 0.05 m.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                // The Pinpoint's built-in IMU supplies heading; refresh it each loop.
                odo.update();
                double headingDegrees = odo.getHeading(AngleUnit.DEGREES);
                // Pinpoint reports counterclockwise-positive; drivetrain turn is clockwise-positive.
                clockwiseTurnRateDegrees = -odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
                limelight.updateRobotOrientation(headingDegrees + FIELD_HEADING_OFFSET_DEGREES);
                telemetry.addData("Pinpoint heading (deg)", headingDegrees);
                telemetry.addData("Clockwise turn rate (deg/s)", clockwiseTurnRateDegrees);

                LLResult result = limelight.getLatestResult();
                updateAlignment(result, System.nanoTime());
                telemetry.addData("Alignment", alignmentStatus);
                if (result != null && result.isValid()) {
                    telemetry.addData("Target_X", result.getTx());
                    telemetry.addData("Target_Y", result.getTy());
                    telemetry.addData("Pipeline: ", result.getPipelineIndex());

                    List<FiducialResult> fiducials = result.getFiducialResults();
                    if (fiducials != null) {
                        for (FiducialResult fiducial : fiducials) {
                            int id = fiducial.getFiducialId();
                            double x = fiducial.getTargetXDegrees();
                            double y = fiducial.getTargetYDegrees();
                            String label = "Fiducial " + id;
                            telemetry.addData(label + " angles (deg)", "X: %.2f, Y: %.2f", x, y);

                            Pose3D robotPoseTargetSpace = fiducial.getRobotPoseTargetSpace();
                            if (robotPoseTargetSpace != null) {
                                Position position = robotPoseTargetSpace.getPosition().toUnit(DistanceUnit.METER);
                                // Target-space X is lateral, Y is vertical, and Z is normal to the tag.
                                double strafeDistance3D = position.x;
                                double verticalDistance3D = position.y;
                                double distance = Math.sqrt(position.x * position.x
                                        + position.y * position.y + position.z * position.z);
                                telemetry.addData(label, "is %.3f meters away", distance);
                                telemetry.addData(label + "cell-relative lateral offset (m)", "%.3f", strafeDistance3D);
                                telemetry.addData(label + "cell-relative vertical offset (m)", "%.3f", verticalDistance3D);
                                telemetry.addData(label + "AprilTag Area: ", fiducial.getTargetArea());
                            }
                        }
                    }

                    Pose3D botpose = result.getBotpose_MT2();
                    if (botpose != null) {
                        double x = botpose.getPosition().x;
                        double y = botpose.getPosition().y;
                        telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                        telemetry.addData("Botpose", botpose.toString());
                    }
                } else {
                    telemetry.addLine("No AprilTag result available.");
                }
                telemetry.update();
                idle();
            }
        } finally {
            setDrivePower(0.0, 0.0);
            limelight.stop();
        }

    }

    private void initializeDrive() {
        frontLeft = hardwareMap.get(DcMotor.class, "FLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FRight");
        backLeft = hardwareMap.get(DcMotor.class, "BLeft");
        backRight = hardwareMap.get(DcMotor.class, "BRight");
        // Match the working drivetrain configuration in Emily.java.
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotor motor : new DcMotor[] {frontLeft, frontRight, backLeft, backRight}) {
            motor.setPower(0.0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void updateAlignment(LLResult result, long nowNs) {
        boolean aPressed = gamepad1.a && !aWasPressed;
        aWasPressed = gamepad1.a;
        if (result != null && Double.isFinite(result.getTimestamp())
                && result.getTimestamp() != lastCameraTimestamp) {
            lastCameraTimestamp = result.getTimestamp();
            lastNewFrameNs = nowNs;
        }

        if (gamepad1.b || (aPressed && aligning)) {
            stopAlignment("Cancelled. Press A to try again.");
            return;
        }
        if (!aligning && !aPressed) {
            setDrivePower(0.0, 0.0);
            return;
        }
        if (!Double.isFinite(clockwiseTurnRateDegrees)) {
            stopAlignment("Stopped: invalid Pinpoint turn rate.");
            return;
        }

        if (result == null || !result.isValid()
                || result.getPipelineIndex() != APRILTAG_PIPELINE
                || result.getStaleness() < 0 || result.getStaleness() > MAX_RESULT_AGE_MS
                || !Double.isFinite(result.getTimestamp())
                || nowNs - lastNewFrameNs > MAX_RESULT_AGE_MS * 1_000_000L) {
            stopAlignment("Stopped: no fresh AprilTag result. Press A to retry.");
            return;
        }

        List<FiducialResult> fiducials = result.getFiducialResults();
        // The FTC SDK does not expose the primary tag ID for result.getTx().
        // Require one tag so POI tx and the measured distance refer to the same target.
        if (fiducials == null || fiducials.size() != 1) {
            stopAlignment("Stopped: alignment requires exactly one visible tag.");
            return;
        }
        FiducialResult tag = fiducials.get(0);
        Pose3D pose = tag.getRobotPoseTargetSpace();
        double tx = result.getTx(); // Uses the active pipeline's configured POI/crosshair.
        if (pose == null || !Double.isFinite(tx)) {
            stopAlignment("Stopped: missing target pose or tx.");
            return;
        }
        Position position = pose.getPosition().toUnit(DistanceUnit.METER);
        double distance = Math.sqrt(position.x * position.x
                + position.y * position.y + position.z * position.z);
        // Missing Limelight 3D transforms can be returned as an all-zero pose.
        if (!Double.isFinite(distance) || distance <= 0.0) {
            stopAlignment("Stopped: invalid 3D distance. Check Full 3D and camera setup.");
            return;
        }

        if (!aligning) {
            aligning = true;
            alignmentTagId = tag.getFiducialId();
            alignmentStartedNs = nowNs;
            inToleranceSinceNs = 0;
        }
        if (tag.getFiducialId() != alignmentTagId) {
            stopAlignment("Stopped: target tag changed. Press A to retry.");
            return;
        }
        if (nowNs - alignmentStartedNs >= ALIGNMENT_TIMEOUT_NS) {
            stopAlignment("Stopped: alignment timed out. Press A to retry.");
            return;
        }

        double distanceError = distance - TARGET_DISTANCE_METERS;
        boolean txAligned = Math.abs(tx) <= TX_TOLERANCE_DEGREES;
        boolean distanceAligned = Math.abs(distanceError) <= DISTANCE_TOLERANCE_METERS;
        telemetry.addData("Alignment tag", alignmentTagId);
        telemetry.addData("Alignment tx / distance", "%.2f deg / %.3f m", tx, distance);
        telemetry.addData("Distance error (m)", "%.3f", distanceError);

        if (txAligned && distanceAligned) {
            setDrivePower(0.0, 0.0);
            if (inToleranceSinceNs == 0) inToleranceSinceNs = nowNs;
            alignmentStatus = "Settling within both tolerances.";
            if (nowNs - inToleranceSinceNs >= SETTLE_TIME_NS) {
                stopAlignment("Complete: tx and distance within tolerance. Press A to repeat.");
            }
            return;
        }
        inToleranceSinceNs = 0;
        // Correct small angle and distance errors together instead of repeatedly
        // blocking translation whenever tx drifts just outside the final tolerance.
        double turn = calculateTurnPower(tx, clockwiseTurnRateDegrees);
        boolean canAdjustDistance = Math.abs(tx) <= MAX_DRIVE_TX_DEGREES;
        double forward = canAdjustDistance && !distanceAligned
                ? proportionalPower(distanceError, DRIVE_GAIN, MIN_DRIVE_POWER, MAX_DRIVE_POWER)
                : 0.0;
        setDrivePower(forward, turn);
        alignmentStatus = forward != 0.0
                ? "Adjusting distance and correcting tx." : "Turning toward POI.";
        telemetry.addData("Auto forward / turn", "%.2f / %.2f", forward, turn);
    }

    private void stopAlignment(String status) {
        setDrivePower(0.0, 0.0);
        aligning = false;
        inToleranceSinceNs = 0;
        alignmentStatus = status;
    }

    private static double proportionalPower(double error, double gain, double minimum, double maximum) {
        double magnitude = Range.clip(Math.abs(error) * gain, minimum, maximum);
        return Math.copySign(magnitude, error);
    }

    private static double calculateTurnPower(double tx, double clockwiseRateDegrees) {
        double errorDegrees = Math.abs(tx);
        if (errorDegrees <= TX_TOLERANCE_DEGREES) return 0.0;

        // Keep the configured minimum for large turns, but taper it toward zero
        // near the tolerance instead of kicking across the crosshair at 20% power.
        double minimumScale = Range.clip(
                (errorDegrees - TX_TOLERANCE_DEGREES)
                        / (TURN_SLOWDOWN_DEGREES - TX_TOLERANCE_DEGREES), 0.0, 1.0);
        double approachPower = Math.copySign(
                Range.clip(Math.max(errorDegrees * TURN_GAIN, MIN_TURN_POWER * minimumScale),
                        0.0, MAX_TURN_POWER), tx);
        double dampedPower = approachPower - TURN_DAMPING_GAIN * clockwiseRateDegrees;
        // Brake if already turning toward the target too quickly. Reverse only
        // when the measured target error changes sign, not just from gyro damping.
        if (dampedPower * tx <= 0.0) return 0.0;
        return Range.clip(dampedPower, -MAX_TURN_POWER, MAX_TURN_POWER);
    }

    private void setDrivePower(double forward, double clockwiseTurn) {
        // Positive tx is right of the crosshair, so a positive turn is clockwise.
        double left = forward + clockwiseTurn;
        double right = forward - clockwiseTurn;
        double scale = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        frontLeft.setPower(left / scale);
        backLeft.setPower(left / scale);
        frontRight.setPower(right / scale);
        backRight.setPower(right / scale);
    }
}
