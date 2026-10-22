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
                telemetry.addLine("Requires AprilTag pipeline 0, correct field map, Full 3D, camera mount pose.");
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
                        }
                        telemetry.addData("Visible tag IDs", ids.toString());
                        Pose3D pose = result.getBotpose();
                        if (pose != null) {
                            Position position = pose.getPosition().toUnit(DistanceUnit.INCH);
                            telemetry.addData("Field X / Y / Z (in)", "%.2f / %.2f / %.2f",
                                    position.x, position.y, position.z);
                            telemetry.addData("Field yaw (deg)",
                                    pose.getOrientation().getYaw(AngleUnit.DEGREES));
                            telemetry.addLine("MegaTag1, field-map coordinates; NOT Pedro coordinates.");
                            telemetry.addLine("Verify at measured positions before using for autonomous.");
                        } else {
                            telemetry.addLine("Tags detected but no robot pose. Check Full 3D and field map.");
                        }
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
}
