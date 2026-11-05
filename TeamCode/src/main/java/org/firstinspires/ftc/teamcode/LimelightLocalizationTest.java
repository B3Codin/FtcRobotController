package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;
import java.util.Map;
import java.util.ArrayDeque;

/**
 * Smart Group aiming and distance test. Press A after START to move the drivetrain.
 * In pipeline 0, enable Smart Target Grouping, select center targeting, and zero
 * the 3D POI offset to aim at the group center. Use ID filters to choose its members.
 * Pinpoint supplies the follower pose; vision updates a relative hold target.
 * Vision does not reset the robot's pose or require a field map.
 */
@TeleOp(name = "Limelight Localization Test", group = "Sensor")
public class LimelightLocalizationTest extends LinearOpMode {
    private static final double TARGET_DISTANCE_METERS = 3.0;
    private static final double DISTANCE_TOLERANCE_METERS = 0.05;
    private static final double TX_TOLERANCE_DEGREES = 1.0;
    private static final double MAX_DRIVE_TX_DEGREES = 10.0;
    private static final double MAX_POWER = 0.8;
    private static final double CAMERA_FORWARD_METERS = 0.05;
    private static final double CAMERA_RIGHT_METERS = 0.0;
    private static final double CAMERA_UP_METERS = 0.2;
    // Positive yaw is left/CCW; positive pitch looks up; positive roll tilts camera-right down.
    private static final double CAMERA_YAW_DEGREES = 0.0;
    // The mount is 60 degrees from vertical, i.e. 30 degrees above horizontal.
    private static final double CAMERA_PITCH_DEGREES = 30.0;
    private static final double CAMERA_ROLL_DEGREES = 0.0;
    private static final long ALIGNMENT_TIMEOUT_NS = 2_000_000_000L;
    private static final int APRILTAG_PIPELINE = 0;
    private static final long MAX_STALENESS_MS = 100;
    private static final long POSE_HISTORY_NS = 1_000_000_000L;
    private static final long STATUS_REFRESH_NS = 1_000_000_000L;

    private Follower follower;
    private Pose targetPose;
    // Display snapshots only: these never command the follower after a dropout/stop.
    private Pose lastTargetPose;
    private double lastTx = Double.NaN;
    private double lastDistanceMeters = Double.NaN;
    private double lastCaptureLatencyMs = Double.NaN;
    private double lastVisionDelayMs = Double.NaN;
    private long lastUsableFrameNs;
    private boolean aligning;
    private boolean aWasPressed;
    private long alignmentStartedNs;
    private double lastCameraTimestamp = Double.NaN;
    private long lastNewFrameNs;
    private double previousTargetFrameTimestamp = Double.NaN;
    private final ArrayDeque<TimedPose> poseHistory = new ArrayDeque<>();
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

    private static final class RobotVector {
        final double forward, left, up;

        RobotVector(double forward, double left, double up) {
            this.forward = forward;
            this.left = left;
            this.up = up;
        }
    }

    private static final class TimedPose {
        final long timeNs;
        final Pose pose;

        TimedPose(long timeNs, Pose pose) {
            this.timeNs = timeNs;
            this.pose = pose;
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
        Follower configured = Constants.create(hardwareMap);
        // Keep the factory's Pinpoint and Foresight instances, with a local power cap.
        follower = new Follower(configured.localizer,
                new PowerLimitedDrivetrain(configured.drivetrain), configured.algorithm());
        stopFollower();

        telemetry.setMsTransmissionInterval(100);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        try {
            limelight.start();
            LLStatus status = null;
            long lastStatusRefreshNs = 0;
            telemetry.addLine("Pipeline 0: enable Smart Target Grouping and center targeting; zero POI offset.");
            telemetry.update();
            // Show measurements during INIT as well as after START.
            while (!isStopRequested()) {
                // getStatus() is a blocking HTTP request. Never issue it while driving.
                long statusTimeNs = System.nanoTime();
                if (!aligning && (status == null || statusTimeNs - lastStatusRefreshNs >= STATUS_REFRESH_NS)) {
                    status = limelight.getStatus();
                    lastStatusRefreshNs = System.nanoTime();
                }
                if (status != null) {
                    telemetry.addData("Camera", status.getName());
                    telemetry.addData("Pipeline", "%d (%s)",
                            status.getPipelineIndex(), status.getPipelineType());
                    telemetry.addData("FPS (last idle status)", status.getFps());
                }

                // Record odometry after any HTTP request, immediately before reading cached vision.
                follower.localizer.update();
                LLResult result = limelight.getLatestResult();
                long nowNs = System.nanoTime();
                recordPose(nowNs, follower.pose());
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
                    double captureLatencyMs = result.getCaptureLatency() + result.getTargetingLatency();
                    if (Double.isFinite(captureLatencyMs)) lastCaptureLatencyMs = captureLatencyMs;
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
                        if (measurement != null && Double.isFinite(result.getTx())) {
                            lastTx = result.getTx();
                            lastDistanceMeters = measurement.distanceMeters;
                            lastUsableFrameNs = lastNewFrameNs;
                        }
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
                    stopFollower();
                }
                // Full Foresight hold control runs every loop, including between camera frames.
                follower.update();
                telemetry.addData("Alignment", alignmentStatus);
                telemetry.addData("Pinpoint follower pose", follower.pose());
                telemetry.addData("Pinpoint turn rate (deg/s)", "%.1f",
                        Math.toDegrees(follower.velocity().omega));
                telemetry.addData("Pedro target state", follower.holding() ? "ACTIVE" : "STOPPED");
                // Repeat the last snapshots every loop so they remain readable after
                // an intermittent detection, timeout, or cancellation. Label them as last values.
                if (lastTargetPose != null) {
                    telemetry.addData("Last Pedro target XY (in)", "%.2f, %.2f",
                            lastTargetPose.x(), lastTargetPose.y());
                    telemetry.addData("Last Pedro target heading (deg)", "%.2f",
                            Math.toDegrees(normalizeAngle(lastTargetPose.heading())));
                    telemetry.addData("Heading error to last target (deg)", "%.2f",
                            Math.toDegrees(normalizeAngle(lastTargetPose.heading() - follower.pose().heading())));
                }
                if (Double.isFinite(lastTx)) {
                    telemetry.addData("Last usable tx / distance", "%.2f deg / %.3f m", lastTx, lastDistanceMeters);
                    telemetry.addData("Last usable frame received (ms ago)", (nowNs - lastUsableFrameNs) / 1_000_000L);
                }
                if (Double.isFinite(lastCaptureLatencyMs)) {
                    telemetry.addData("Last capture + targeting latency (ms)", "%.1f", lastCaptureLatencyMs);
                }
                if (Double.isFinite(lastVisionDelayMs)) {
                    telemetry.addData("Last compensated vision delay (ms)", "%.1f", lastVisionDelayMs);
                }
                telemetry.update();
                sleep(20);
            }
        } finally {
            stopFollower();
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

    private void updateAlignment(LLResult result, GroupMeasurement measurement, long nowNs) {
        boolean aPressed = gamepad1.a && !aWasPressed;
        aWasPressed = gamepad1.a;
        if (gamepad1.b || (aPressed && aligning)) {
            stopAlignment("Cancelled. Press A to repeat.");
            return;
        }
        if (!aligning && !aPressed) {
            stopFollower();
            return;
        }
        if (!aligning) {
            aligning = true;
            alignmentStartedNs = nowNs;
            stopFollower();
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
            stopFollower();
            alignmentStatus = "Paused: waiting for fresh Smart Group angles and 3D distance.";
            return;
        }

        double tx = result.getTx();
        double distanceError = measurement.distanceMeters - TARGET_DISTANCE_METERS;
        boolean atHoldTarget = targetPose == null || (
                Math.abs(normalizeAngle(targetPose.heading() - follower.pose().heading()))
                        <= Math.toRadians(TX_TOLERANCE_DEGREES)
                && targetPose.distance(follower.pose())
                        <= DistanceUnit.INCH.fromMeters(DISTANCE_TOLERANCE_METERS));
        if (Math.abs(tx) <= TX_TOLERANCE_DEGREES
                && Math.abs(distanceError) <= DISTANCE_TOLERANCE_METERS && atHoldTarget) {
            stopAlignment("Complete: tx and distance within tolerance. Press A to repeat.");
            return;
        }
        if (targetPose == null || result.getTimestamp() != previousTargetFrameTimestamp) {
            double latencyMs = result.getCaptureLatency() + result.getTargetingLatency()
                    + result.getStaleness();
            if (!Double.isFinite(latencyMs) || latencyMs < 0.0
                    || latencyMs * 1_000_000.0 > POSE_HISTORY_NS) {
                stopFollower();
                alignmentStatus = "Paused: invalid or excessive camera latency.";
                return;
            }
            Pose capturePose = poseAt(nowNs - Math.round(latencyMs * 1_000_000.0));
            if (capturePose == null) {
                stopFollower();
                alignmentStatus = "Paused: waiting for camera-time Pinpoint history.";
                return;
            }
            lastVisionDelayMs = latencyMs;
            Pose nextTarget = calculateTargetPose(capturePose, measurement,
                    tx, result.getTxNC(), result.getTyNC());
            if (nextTarget == null) {
                stopFollower();
                alignmentStatus = "Paused: invalid camera geometry or unreachable 3 m camera range.";
                return;
            }
            // Turn-only holds must use today's position, not the earlier capture position.
            if (Math.abs(tx) > MAX_DRIVE_TX_DEGREES
                    || Math.abs(distanceError) <= DISTANCE_TOLERANCE_METERS) {
                nextTarget = new Pose(follower.pose().x(), follower.pose().y(), nextTarget.heading());
            }
            previousTargetFrameTimestamp = result.getTimestamp();
            // Keep a fixed odometry target through vision jitter smaller than our
            // requested tolerances, so Foresight can actually brake into that target.
            boolean changed = targetPose == null || nextTarget.distance(targetPose)
                    > DistanceUnit.INCH.fromMeters(DISTANCE_TOLERANCE_METERS)
                    || Math.abs(normalizeAngle(nextTarget.heading() - targetPose.heading()))
                    > Math.toRadians(TX_TOLERANCE_DEGREES);
            if (changed) {
                if (targetPose == null) follower.algorithm().reset();
                targetPose = nextTarget;
                lastTargetPose = nextTarget;
                // hold() preserves Foresight's controller/braking state.
                follower.hold(targetPose);
            }
        }
        alignmentStatus = Math.abs(tx) <= MAX_DRIVE_TX_DEGREES
                ? "Pedro: approaching vision target." : "Pedro: turning toward Smart Group.";
    }

    private void stopAlignment(String status) {
        stopFollower();
        aligning = false;
        alignmentStatus = status;
    }

    private void stopFollower() {
        targetPose = null;
        previousTargetFrameTimestamp = Double.NaN;
        if (follower != null) {
            follower.stop();
            // Follower.stop() changes mode; explicitly stop hardware immediately too.
            follower.drivetrain.stop(true);
        }
    }

    private void recordPose(long timeNs, Pose pose) {
        poseHistory.addLast(new TimedPose(timeNs, pose));
        // Retain one sample before the window for interpolation at its boundary.
        while (poseHistory.size() > 2) {
            TimedPose first = poseHistory.removeFirst();
            if (poseHistory.peekFirst().timeNs > timeNs - POSE_HISTORY_NS) {
                poseHistory.addFirst(first);
                break;
            }
        }
    }

    private Pose poseAt(long timeNs) {
        if (poseHistory.isEmpty() || timeNs < poseHistory.peekFirst().timeNs) return null;
        TimedPose previous = poseHistory.peekFirst();
        for (TimedPose next : poseHistory) {
            if (next.timeNs >= timeNs) {
                if (next.timeNs == previous.timeNs) return previous.pose;
                double fraction = (double) (timeNs - previous.timeNs) / (next.timeNs - previous.timeNs);
                double heading = previous.pose.heading() + fraction
                        * normalizeAngle(next.pose.heading() - previous.pose.heading());
                return new Pose(previous.pose.x() + fraction * (next.pose.x() - previous.pose.x()),
                        previous.pose.y() + fraction * (next.pose.y() - previous.pose.y()), heading);
            }
            previous = next;
        }
        return previous.pose;
    }

    private static RobotVector cameraToRobot(double right, double down, double forward) {
        double roll = Math.toRadians(CAMERA_ROLL_DEGREES);
        double pitch = Math.toRadians(CAMERA_PITCH_DEGREES);
        double yaw = Math.toRadians(CAMERA_YAW_DEGREES);
        double rolledRight = Math.cos(roll) * right - Math.sin(roll) * down;
        double rolledDown = Math.sin(roll) * right + Math.cos(roll) * down;
        double robotForward = Math.cos(pitch) * forward + Math.sin(pitch) * rolledDown;
        double robotLeft = -rolledRight;
        double robotUp = Math.sin(pitch) * forward - Math.cos(pitch) * rolledDown;
        return new RobotVector(Math.cos(yaw) * robotForward - Math.sin(yaw) * robotLeft,
                Math.sin(yaw) * robotForward + Math.cos(yaw) * robotLeft, robotUp);
    }

    private static Pose calculateTargetPose(Pose robot, GroupMeasurement measurement,
                                            double tx, double txPrincipal, double tyPrincipal) {
        if (!Double.isFinite(robot.x()) || !Double.isFinite(robot.y())
                || !Double.isFinite(robot.heading()) || !Double.isFinite(tx)
                || !Double.isFinite(txPrincipal) || !Double.isFinite(tyPrincipal)) return null;

        // Use the actual Smart Group ray for heading, not the 3D centroid's bearing.
        // Principal-pixel angles also retain any calibrated crosshair offset.
        RobotVector observedRay = cameraToRobot(Math.tan(Math.toRadians(txPrincipal)),
                -Math.tan(Math.toRadians(tyPrincipal)), 1.0);
        RobotVector centeredRay = cameraToRobot(Math.tan(Math.toRadians(txPrincipal - tx)),
                -Math.tan(Math.toRadians(tyPrincipal)), 1.0);
        if (Math.hypot(observedRay.forward, observedRay.left) < 1e-6
                || Math.hypot(centeredRay.forward, centeredRay.left) < 1e-6) return null;
        double headingCorrection = Math.abs(tx) <= TX_TOLERANCE_DEGREES ? 0.0
                : normalizeAngle(Math.atan2(observedRay.left, observedRay.forward)
                        - Math.atan2(centeredRay.left, centeredRay.forward));
        double heading = normalizeAngle(robot.heading() + headingCorrection);

        // The configured Pinpoint/Foresight coordinates are inches; geometry below uses meters.
        double robotX = DistanceUnit.INCH.toMeters(robot.x());
        double robotY = DistanceUnit.INCH.toMeters(robot.y());
        double goalX = robotX, goalY = robotY;
        if (Math.abs(tx) <= MAX_DRIVE_TX_DEGREES
                && Math.abs(measurement.distanceMeters - TARGET_DISTANCE_METERS) > DISTANCE_TOLERANCE_METERS) {
            RobotVector relative = cameraToRobot(measurement.x, measurement.y, measurement.z);
            double c = Math.cos(robot.heading()), s = Math.sin(robot.heading());
            double cameraLeft = -CAMERA_RIGHT_METERS;
            double cameraX = robotX + c * CAMERA_FORWARD_METERS - s * cameraLeft;
            double cameraY = robotY + s * CAMERA_FORWARD_METERS + c * cameraLeft;
            double groupX = cameraX + c * relative.forward - s * relative.left;
            double groupY = cameraY + s * relative.forward + c * relative.left;
            double groupHeight = CAMERA_UP_METERS + relative.up;
            double heightDifference = groupHeight - CAMERA_UP_METERS;
            double radiusSquared = TARGET_DISTANCE_METERS * TARGET_DISTANCE_METERS
                    - heightDifference * heightDifference;
            double currentRadius = Math.hypot(groupX - cameraX, groupY - cameraY);
            if (radiusSquared <= 0.0 || currentRadius < 1e-6) return null;

            // Keep the measured approach direction, solving for the horizontal range
            // that gives a 3 m camera-to-group slant distance at this target height.
            double radiusScale = Math.sqrt(radiusSquared) / currentRadius;
            double goalCameraX = groupX - (groupX - cameraX) * radiusScale;
            double goalCameraY = groupY - (groupY - cameraY) * radiusScale;
            goalX = goalCameraX - Math.cos(heading) * CAMERA_FORWARD_METERS
                    + Math.sin(heading) * cameraLeft;
            goalY = goalCameraY - Math.sin(heading) * CAMERA_FORWARD_METERS
                    - Math.cos(heading) * cameraLeft;
        }
        if (!Double.isFinite(goalX) || !Double.isFinite(goalY) || !Double.isFinite(heading)) return null;
        return new Pose(DistanceUnit.INCH.fromMeters(goalX), DistanceUnit.INCH.fromMeters(goalY), heading);
    }

    private static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    /** Limits final mecanum wheel power while keeping Pedro's allocation and low-power braking. */
    private static final class PowerLimitedDrivetrain implements Drivetrain {
        private final Drivetrain delegate;

        PowerLimitedDrivetrain(Drivetrain delegate) {
            this.delegate = delegate;
        }

        @Override
        public void drive(DrivePowers powers, boolean manual) {
            double peak = Math.abs(powers.forward()) + Math.abs(powers.strafe()) + Math.abs(powers.turn());
            if (!Double.isFinite(peak)) {
                delegate.stop(true);
                return;
            }
            double scale = peak > MAX_POWER ? MAX_POWER / peak : 1.0;
            delegate.drive(scaled(powers, scale), manual);
        }

        @Override
        public double maxScaling(DrivePowers current, DrivePowers delta) {
            return delegate.maxScaling(scaled(current, 1.0 / MAX_POWER), scaled(delta, 1.0 / MAX_POWER));
        }

        @Override
        public double interpolateVelocity(double xRadius, double yRadius, double theta) {
            return delegate.interpolateVelocity(xRadius, yRadius, theta);
        }

        @Override
        public void stop() { delegate.stop(); }

        @Override
        public void stop(boolean brake) { delegate.stop(brake); }

        @Override
        public Map<String, Object> debug() { return delegate.debug(); }

        private static DrivePowers scaled(DrivePowers powers, double scale) {
            return new DrivePowers(powers.forward() * scale, powers.strafe() * scale, powers.turn() * scale);
        }
    }
}
