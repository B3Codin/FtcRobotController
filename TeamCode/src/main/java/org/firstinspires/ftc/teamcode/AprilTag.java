package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

@TeleOp (name = "AprilTag")
public class AprilTag extends LinearOpMode {
    // Adjust so Pinpoint heading + this offset matches the Limelight field map's yaw.
    private static final double FIELD_HEADING_OFFSET_DEGREES = 0.0;

    private Limelight3A limelight;
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        try {
            limelight.start();
            telemetry.addLine("Ready. Pinpoint heading must match the Limelight field map.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                // The Pinpoint's built-in IMU supplies heading; refresh it each loop.
                odo.update();
                double headingDegrees = odo.getHeading(AngleUnit.DEGREES);
                limelight.updateRobotOrientation(headingDegrees + FIELD_HEADING_OFFSET_DEGREES);
                telemetry.addData("Pinpoint heading (deg)", headingDegrees);

                LLResult result = limelight.getLatestResult();
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
            limelight.stop();
        }

    }
}
