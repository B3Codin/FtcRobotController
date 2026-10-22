package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/** Stationary camera setup test. Does not initialize or command the drivetrain. */
@TeleOp(name = "Limelight Localization Test", group = "Sensor")
public class LimelightLocalizationTest extends LinearOpMode {
    private static final int APRILTAG_PIPELINE = 0;
    private static final long MAX_STALENESS_MS = 100;

    // BIOBUZZ manual section 9.9. A visible member identifies its entire cluster.
    // Limelight returns member tags, not FTC AprilTagClusterDetection objects.
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


        telemetry.setMsTransmissionInterval(100);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        try {
            limelight.start();
            // Show measurements during INIT as well as after START.
            while (!isStopRequested()) {
                telemetry.addLine("Stationary localization test; no motor commands.");
                telemetry.addLine("Requires AprilTag pipeline 0, correct tag size, and Full 3D.");
                telemetry.addLine("Camera-relative: X forward, Y left; heading about camera up.");
                LLStatus status = limelight.getStatus();
                if (status != null) {
                    telemetry.addData("Camera", status.getName());
                    telemetry.addData("Pipeline", "%d (%s)",
                            status.getPipelineIndex(), status.getPipelineType());
                    telemetry.addData("FPS", status.getFps());
                }

                LLResult result = limelight.getLatestResult();
                if (result == null) {
                    telemetry.addLine("Waiting for camera data. Check USB, power, and robot configuration.");
                } else {
                    telemetry.addData("Result age (ms)", result.getStaleness());
                    telemetry.addData("Capture + targeting latency (ms)",
                            result.getCaptureLatency() + result.getTargetingLatency());
                    boolean usable = result.isValid()
                            && result.getPipelineIndex() == APRILTAG_PIPELINE
                            && result.getStaleness() < MAX_STALENESS_MS
                            && result.getFiducialResults() != null
                            && !result.getFiducialResults().isEmpty();
                    if (usable) {
                        int clustersVisible = 0;
                        for (Cluster cluster : Cluster.values()) {
                            LLResultTypes.FiducialResult referenceTag = null;
                            int visibleMembers = 0;
                            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                                if (!cluster.contains(tag.getFiducialId())) continue;
                                visibleMembers++;
                                if (referenceTag == null
                                        || tag.getTargetArea() > referenceTag.getTargetArea()) {
                                    referenceTag = tag;
                                }
                            }
                            if (referenceTag != null) {
                                clustersVisible++;
                                telemetry.addData(cluster.label + " cluster",
                                        "%d/4 members visible; reference tag %d",
                                        visibleMembers, referenceTag.getFiducialId());
                                reportCameraOffsets(cluster.label, referenceTag);
                            }
                        }
                        telemetry.addData("Visible BIOBUZZ clusters", clustersVisible);
                        if (clustersVisible == 0) {
                            telemetry.addLine("Detected tags do not belong to BIOBUZZ clusters (IDs 30-45).");
                        }
                        telemetry.addLine("Cluster identity only: offsets use its reference tag, not CELL opening.");
                        telemetry.addLine("Cluster identity alone does not confirm the CELL is pointed up.");
                        telemetry.addLine("Camera pitch/roll must be accounted for before using ground distances.");
                    } else {
                        telemetry.addLine("No fresh AprilTag result from pipeline 0.");
                    }
                }
                telemetry.update();
                sleep(20);
            }
        } finally {
            limelight.stop();
        }
    }

    private void reportCameraOffsets(String clusterLabel, LLResultTypes.FiducialResult tag) {
        String label = clusterLabel + " reference";
        Pose3D pose = tag.getTargetPoseCameraSpace();
        if (pose == null) {
            telemetry.addData(label, "No camera-relative 3D pose");
            return;
        }

        // Native Limelight camera axes: +X right, +Y down, +Z forward.
        // Remap into a camera-relative forward/left/up frame. These are not field axes.
        Position position = pose.getPosition().toUnit(DistanceUnit.MM);
        double xForwardMm = position.z;
        double yLeftMm = -position.x;
        double zUpMm = -position.y;
        // The SDK stores the native rotation about camera Y in getPitch(), not getYaw().
        // Negate it to express heading about camera up rather than camera down.
        double headingDegrees = -pose.getOrientation().getPitch(AngleUnit.DEGREES);
        if (!Double.isFinite(xForwardMm) || !Double.isFinite(yLeftMm)
                || !Double.isFinite(zUpMm) || !Double.isFinite(headingDegrees)
                || xForwardMm <= 0) {
            telemetry.addData(label, "Invalid/missing 3D pose; check Full 3D and tag size");
            return;
        }

        telemetry.addData(label + " X forward (mm)", "%.1f", xForwardMm);
        telemetry.addData(label + " Y left (mm)", "%.1f", yLeftMm);
        telemetry.addData(label + " Z up (mm)", "%.1f", zUpMm);
        telemetry.addData(label + " heading offset (deg)", "%.1f", headingDegrees);
        // Bearing is the turn toward the tag center; tag orientation is a separate quantity.
        telemetry.addData(label + " bearing left (deg)", "%.1f",
                Math.toDegrees(Math.atan2(yLeftMm, xForwardMm)));
        telemetry.addData(label + " camera-plane range (mm)", "%.1f",
                Math.hypot(xForwardMm, yLeftMm));
    }
}
