package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * Smart Group aiming and distance test. Press A after START to move the drivetrain.
 * In pipeline 0, enable Smart Target Grouping, select center targeting, and zero
 * the 3D POI offset to aim at the group center. Use ID filters to choose its members.
 */
@TeleOp(name = "Limelight Localization Test", group = "Sensor")
public class LimelightLocalizationTest extends LinearOpMode {
    // PD gains use degrees for turning and meters for forward/backward distance.
    private static final double TURN_P = 0.025;
    private static final double TURN_D = 0.0;
    private static final double DRIVE_P = 0.25;
    private static final double DRIVE_D = 0.0;
    private static final double TARGET_DISTANCE_METERS = 3.0;
    private static final double DISTANCE_TOLERANCE_METERS = 0.05;
    private static final double TX_TOLERANCE_DEGREES = 1.0;
    private static final double MAX_DRIVE_TX_DEGREES = 10.0;
    private static final double MIN_POWER = 0.15;
    private static final double MAX_POWER = 0.8;
    private static final long ALIGNMENT_TIMEOUT_NS = 2_000_000_000L;
    private static final int APRILTAG_PIPELINE = 0;
    private static final long MAX_STALENESS_MS = 100;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private boolean aligning;
    private boolean aWasPressed;
    private long alignmentStartedNs;
    private double lastCameraTimestamp = Double.NaN;
    private long lastNewFrameNs;
    private boolean havePreviousErrors;
    private double previousErrorFrameTimestamp;
    private long previousErrorSampleNs;
    private double previousTx, previousDistanceError;
    private double txRate, distanceErrorRate;
    private String alignmentStatus = "Ready: A starts alignment; A again or B cancels.";

    private static final class GroupMeasurement {
        final double x, y, z, distanceMeters;
        final int tagCount;

        GroupMeasurement(double x, double y, double z, double distanceMeters, int tagCount) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.distanceMeters = distanceMeters;
            this.tagCount = tagCount;
        }
    }

    // BIOBUZZ membership is informational; the camera's pipeline controls Smart Group targeting.
    private enum Cluster {
        RED_FAR("Red opposite audience", 30),
        RED_AUDIENCE("Red audience", 34),
        BLUE_AUDIENCE("Blue audience", 38),
        BLUE_FAR("Blue opposite audience", 42);

        final String label;
        final int firstTagId;

        Cluster(String label, int firstTagId) {
            this.label = label;
            this.firstTagId = firstTagId;
        }

        boolean contains(int tagId) {
            return tagId >= firstTagId && tagId < firstTagId + 4;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        initializeDrive();

        telemetry.setMsTransmissionInterval(100);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        try {
            limelight.start();
            telemetry.addLine("Pipeline 0: enable Smart Target Grouping and center targeting; zero POI offset.");
            telemetry.update();
            // Show measurements during INIT as well as after START.
            while (!isStopRequested()) {
                LLStatus status = limelight.getStatus();
                if (status != null) {
                    telemetry.addData("Camera", status.getName());
                    telemetry.addData("Pipeline", "%d (%s)",
                            status.getPipelineIndex(), status.getPipelineType());
                    telemetry.addData("FPS", status.getFps());
                }

                LLResult result = limelight.getLatestResult();
                long nowNs = System.nanoTime();
                if (result != null && Double.isFinite(result.getTimestamp())
                        && result.getTimestamp() != lastCameraTimestamp) {
                    lastCameraTimestamp = result.getTimestamp();
                    lastNewFrameNs = nowNs;
                }
                GroupMeasurement measurement = null;
                if (result == null) {
                    telemetry.addLine("Waiting for camera data. Check USB, power, and robot configuration.");
                } else {
                    telemetry.addData("Result age (ms)", result.getStaleness());
                    telemetry.addData("Capture + targeting latency (ms)",
                            result.getCaptureLatency() + result.getTargetingLatency());
                    boolean usable = result.isValid()
                            && result.getPipelineIndex() == APRILTAG_PIPELINE
                            && result.getStaleness() >= 0
                            && result.getStaleness() < MAX_STALENESS_MS
                            && Double.isFinite(result.getTimestamp())
                            && nowNs - lastNewFrameNs < MAX_STALENESS_MS * 1_000_000L
                            && result.getFiducialResults() != null
                            && !result.getFiducialResults().isEmpty();
                    if (usable) {
                        measurement = estimateGroupMeasurement(result.getFiducialResults());
                        reportSmartGroup(result, measurement);
                        int clustersVisible = 0;
                        for (Cluster cluster : Cluster.values()) {
                            int visibleMembers = 0;
                            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                                if (!cluster.contains(tag.getFiducialId())) continue;
                                visibleMembers++;
                            }
                            if (visibleMembers > 0) {
                                clustersVisible++;
                                telemetry.addData(cluster.label + " cluster",
                                        "%d/4 members visible", visibleMembers);
                            }
                        }
                        telemetry.addData("Visible BIOBUZZ clusters", clustersVisible);
                        if (clustersVisible == 0) {
                            telemetry.addLine("Detected tags do not belong to BIOBUZZ clusters (IDs 30-45).");
                        }
                        if (clustersVisible > 1) {
                            telemetry.addLine("Multiple clusters visible: use pipeline ID filters to choose Smart Group members.");
                        }
                        telemetry.addLine("Smart Group angles follow the pipeline targeting/POI settings.");
                        telemetry.addLine("Center targeting requires Smart Target Grouping and zero POI offset in pipeline 0.");
                    } else {
                        telemetry.addLine("No fresh AprilTag result from pipeline 0.");
                    }
                }
                if (opModeIsActive()) {
                    updateAlignment(result, measurement, nowNs);
                } else {
                    // INIT shows vision telemetry, but cannot command movement.
                    aWasPressed = gamepad1.a;
                    setDrivePower(0.0, 0.0);
                }
                telemetry.addData("Alignment", alignmentStatus);
                telemetry.update();
                sleep(20);
            }
        } finally {
            setDrivePower(0.0, 0.0);
            limelight.stop();
        }
    }

    private void reportSmartGroup(LLResult result, GroupMeasurement measurement) {
        // Read the pipeline's selected group, not any individual FiducialResult.
        // There is one selected Smart Group output per frame, not one per BIOBUZZ cluster.
        double tx = result.getTx();
        double ty = result.getTy();
        if (!Double.isFinite(tx) || !Double.isFinite(ty)) {
            telemetry.addLine("Invalid Smart Group aiming angles.");
            return;
        }
        telemetry.addData("Smart Group tx (deg)", "%.2f", tx);
        telemetry.addData("Smart Group ty (deg)", "%.2f", ty);
        telemetry.addData("Smart Group target area (%)", "%.3f", result.getTa());
        if (measurement == null) {
            telemetry.addLine("No valid 3D group-distance estimate; enable Full 3D and check tag size.");
        } else {
            telemetry.addData("Smart Group estimated distance (m)", "%.3f", measurement.distanceMeters);
            telemetry.addData("Visible-tag center XYZ (m)", "%.3f, %.3f, %.3f",
                    measurement.x, measurement.y, measurement.z);
            telemetry.addData("Tags used for distance estimate", measurement.tagCount);
            telemetry.addLine("Distance is camera-to-visible-tag center; it can shift with visibility.");
        }
    }

    private static GroupMeasurement estimateGroupMeasurement(List<LLResultTypes.FiducialResult> tags) {
        if (tags == null || tags.isEmpty()) return null;
        double x = 0.0, y = 0.0, z = 0.0;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag == null) return null;
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose == null) return null;
            Position position = pose.getPosition().toUnit(DistanceUnit.METER);
            // Native camera axes: X right, Y down, Z forward. All-zero poses mean missing data.
            if (!Double.isFinite(position.x) || !Double.isFinite(position.y)
                    || !Double.isFinite(position.z) || position.z <= 0.0) return null;
            x += position.x;
            y += position.y;
            z += position.z;
        }
        // Average positions first, not individual distances. Pipeline ID filters
        // must restrict these visible tags to the intended Smart Group.
        x /= tags.size();
        y /= tags.size();
        z /= tags.size();
        double distance = Math.sqrt(x * x + y * y + z * z);
        if (!Double.isFinite(distance) || distance <= 0.0) return null;
        return new GroupMeasurement(x, y, z, distance, tags.size());
    }

    private void initializeDrive() {
        frontLeft = hardwareMap.get(DcMotor.class, "FLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FRight");
        backLeft = hardwareMap.get(DcMotor.class, "BLeft");
        backRight = hardwareMap.get(DcMotor.class, "BRight");
        // Match Emily.java and AprilTag.java.
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

    private void updateAlignment(LLResult result, GroupMeasurement measurement, long nowNs) {
        boolean aPressed = gamepad1.a && !aWasPressed;
        aWasPressed = gamepad1.a;
        if (gamepad1.b || (aPressed && aligning)) {
            stopAlignment("Cancelled. Press A to repeat.");
            return;
        }
        if (!aligning && !aPressed) {
            setDrivePower(0.0, 0.0);
            return;
        }
        if (!aligning) {
            aligning = true;
            alignmentStartedNs = nowNs;
            resetDerivative();
        }
        long elapsedNs = nowNs - alignmentStartedNs;
        if (elapsedNs >= ALIGNMENT_TIMEOUT_NS) {
            stopAlignment("Stopped: 2-second timeout. Press A to repeat.");
            return;
        }
        telemetry.addData("Alignment time remaining (s)", "%.2f",
                (ALIGNMENT_TIMEOUT_NS - elapsedNs) / 1_000_000_000.0);
        // A null measurement means missing, invalid, or stale 3D vision data.
        // Stop immediately but remain armed until fresh data or the original timeout.
        if (measurement == null || result == null || !Double.isFinite(result.getTx())) {
            setDrivePower(0.0, 0.0);
            resetDerivative();
            alignmentStatus = "Paused: waiting for fresh Smart Group angles and 3D distance.";
            return;
        }

        double tx = result.getTx();
        double distanceError = measurement.distanceMeters - TARGET_DISTANCE_METERS;
        if (Math.abs(tx) <= TX_TOLERANCE_DEGREES
                && Math.abs(distanceError) <= DISTANCE_TOLERANCE_METERS) {
            stopAlignment("Complete: tx and distance within tolerance. Press A to repeat.");
            return;
        }
        // Only differentiate new camera frames. Repeated reads retain the last
        // derivative instead of alternating between zero and a large spike.
        if (!havePreviousErrors || result.getTimestamp() != previousErrorFrameTimestamp) {
            double dt = (nowNs - previousErrorSampleNs) / 1_000_000_000.0;
            txRate = havePreviousErrors && dt > 0.0 ? (tx - previousTx) / dt : 0.0;
            distanceErrorRate = havePreviousErrors && dt > 0.0
                    ? (distanceError - previousDistanceError) / dt : 0.0;
            previousTx = tx;
            previousDistanceError = distanceError;
            previousErrorSampleNs = nowNs;
            previousErrorFrameTimestamp = result.getTimestamp();
            havePreviousErrors = true;
        }
        double turn = Math.abs(tx) <= TX_TOLERANCE_DEGREES ? 0.0
                : limitedPower(TURN_P * tx + TURN_D * txRate);
        boolean canDrive = Math.abs(tx) <= MAX_DRIVE_TX_DEGREES;
        double forward = canDrive && Math.abs(distanceError) > DISTANCE_TOLERANCE_METERS
                ? limitedPower(DRIVE_P * distanceError + DRIVE_D * distanceErrorRate) : 0.0;
        setDrivePower(forward, turn);
        alignmentStatus = canDrive ? "Correcting tx and distance." : "Turning toward Smart Group.";
        telemetry.addData("Auto forward / clockwise turn", "%.3f / %.3f", forward, turn);
    }

    private void stopAlignment(String status) {
        setDrivePower(0.0, 0.0);
        aligning = false;
        resetDerivative();
        alignmentStatus = status;
    }

    private void resetDerivative() {
        havePreviousErrors = false;
        txRate = 0.0;
        distanceErrorRate = 0.0;
    }

    private static double limitedPower(double power) {
        if (power == 0.0) return 0.0;
        return Math.copySign(Range.clip(Math.abs(power), MIN_POWER, MAX_POWER), power);
    }

    private void setDrivePower(double forward, double clockwiseTurn) {
        double left = forward + clockwiseTurn;
        double right = forward - clockwiseTurn;
        double scale = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)) / MAX_POWER);
        // Bound each nonzero wheel command too, including when drive and turn mix.
        left = limitedPower(left / scale);
        right = limitedPower(right / scale);
        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
    }
}
