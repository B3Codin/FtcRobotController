package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.List;

/** Manual turret test with a one-time magnet zero on negative-side entry. */
@Configurable
@TeleOp(name = "magnetTest", group = "Sensor")
public class magnetTest extends LinearOpMode {
    private static final int ENCODER_PORT = 0;
    private static final double MANUAL_POWER = 0.15;
    // Live in Panels; applied on the next qualifying magnet entry.
    public static volatile double TURRET_MAGNET_ANGLE_DEGREES = 0.0;

    private CRServo leftTurret, rightTurret;
    private DigitalChannel magnet;
    private List<LynxModule> hubs;
    private LynxModule encoderHub;
    private final TurretMagnetZero zero = new TurretMagnetZero();

    @Override
    public void runOpMode() {
        leftTurret = hardwareMap.get(CRServo.class, "leftTurretServo");
        rightTurret = hardwareMap.get(CRServo.class, "rightTurretServo");
        try {
            setTurretPower(0.0);
            magnet = hardwareMap.get(DigitalChannel.class, "limitSwitch");
            magnet.setMode(DigitalChannel.Mode.INPUT);
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) {
                if (!hub.isParent()) {
                    if (encoderHub != null) {
                        throw new IllegalStateException("Multiple Expansion Hubs: select turret encoder Hub explicitly.");
                    }
                    encoderHub = hub;
                }
            }
            if (encoderHub == null) {
                throw new IllegalStateException("Turret encoder Expansion Hub missing; check RS485 connection.");
            }

            telemetry.setMsTransmissionInterval(100);
            sampleAndReport();
            while (opModeInInit()) {
                sampleAndReport();
                idle();
            }
            waitForStart();
            while (opModeIsActive()) {
                boolean lb = gamepad1.left_bumper;
                boolean rb = gamepad1.right_bumper;
                // Both or neither pressed stops; match JackbotsTeleOp's directions.
                setTurretPower(lb == rb ? 0.0 : (lb ? -MANUAL_POWER : MANUAL_POWER));
                sampleAndReport();
                idle();
            }
        } finally {
            setTurretPower(0.0);
        }
    }

    private void sampleAndReport() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        LynxModule.BulkData data = encoderHub.getBulkData();
        if (data.isFake()) {
            throw new IllegalStateException("Turret encoder read failed; stopping OpMode.");
        }
        // Read the shared encoder channel directly; never reset the hardware count.
        int ticks = data.getMotorCurrentPosition(ENCODER_PORT);
        boolean detected = !magnet.getState();
        zero.update(ticks, detected, TURRET_MAGNET_ANGLE_DEGREES);

        telemetry.addData("Turret angle degrees (zeroed)", "%.3f", zero.angleDegrees());
        telemetry.addData("Turret angle degrees (non-zeroed)", "%.3f", zero.unzeroedAngleDegrees());
        telemetry.addData("Magnet detected", detected ? "True" : "False");
        telemetry.addData("raw encoder ticks", ticks);
        telemetry.update();
    }

    private void setTurretPower(double power) {
        leftTurret.setPower(power);
        rightTurret.setPower(power);
    }
}
