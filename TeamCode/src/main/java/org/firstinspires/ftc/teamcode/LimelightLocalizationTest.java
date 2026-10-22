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
                        StringBuilder ids = new StringBuilder();
                        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                            if (ids.length() > 0) ids.append(", ");
                            ids.append(tag.getFiducialId());
                            reportCameraOffsets(tag);
                        }
                        telemetry.addData("Visible tag IDs", ids.toString());
                        telemetry.addLine("Offsets are to individual tags, not the CELL opening or field origin.");
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

    private void reportCameraOffsets(LLResultTypes.FiducialResult tag) {
        String label = "Tag " + tag.getFiducialId();
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
