package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/** Repeats a nominal 180-degree sweep on a goBILDA 2000-0025-0002 in position mode. */
@TeleOp(name = "Servo1Test", group = "Servo")
public class Servo1Test extends LinearOpMode {
    // 300-degree travel across 500-2500 us: 60 to 240 degrees is a 180-degree sweep.
    private static final double BACK_POSITION = 60.0 / 300.0;
    private static final double FORWARD_POSITION = 240.0 / 300.0;
    private static final long MOVE_AND_PAUSE_MS = 1500;

    @Override
    public void runOpMode() {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, "servo1");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        try {
            telemetry.addLine("Ready: servo1 only. Drive motors are not used.");
            telemetry.addLine("Press Start to repeat 180 degrees forward and back.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            // Establish the starting position before the first forward sweep.
            servo.setPosition(BACK_POSITION);
            sleep(MOVE_AND_PAUSE_MS);

            while (opModeIsActive()) {
                servo.setPosition(FORWARD_POSITION);
                telemetry.addLine("servo1: forward 180 degrees");
                telemetry.update();
                sleep(MOVE_AND_PAUSE_MS);
                if (!opModeIsActive()) break;

                servo.setPosition(BACK_POSITION);
                telemetry.addLine("servo1: backward 180 degrees");
                telemetry.update();
                sleep(MOVE_AND_PAUSE_MS);
            }
        } finally {
            servo.setPwmDisable();
        }
    }
}
