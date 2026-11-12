package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/** Runs each flywheel motor at its own target RPM while gamepad1 A is held. */
@TeleOp(name = "flywheelTest", group = "Tuning")
public class flywheelTest extends LinearOpMode {
    private static final double FRONT_TARGET_RPM = -1200;
    private static final double BACK_TARGET_RPM = -1500;
    private static final double RPM_STEP = 100.0;
    private static final DcMotor.Direction FRONT_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction BACK_DIRECTION = DcMotor.Direction.FORWARD;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx front = hardwareMap.get(DcMotorEx.class, "flyFront");
        DcMotorEx back = hardwareMap.get(DcMotorEx.class, "flyBack");
        double frontTargetRPM = FRONT_TARGET_RPM;
        double backTargetRPM = BACK_TARGET_RPM;

        try {
            configureMotor(front, FRONT_DIRECTION);
            configureMotor(back, BACK_DIRECTION);

            // Capture the tuner's current gains at INIT and apply the same set to both motors.
            PIDFCoefficients gains = new PIDFCoefficients(PIDTuning.P, PIDTuning.I, PIDTuning.D, PIDTuning.F);
            for (double value : new double[] {gains.p, gains.i, gains.d, gains.f}) {
                if (!Double.isFinite(value) || value < 0.0) {
                    throw new IllegalArgumentException("PIDTuning gains must be finite and nonnegative.");
                }
            }
            if (gains.p == 0.0 && gains.i == 0.0 && gains.d == 0.0 && gains.f == 0.0) {
                throw new IllegalArgumentException("PIDTuning gains are all zero; set a nonzero F before testing.");
            }
            front.setVelocityPIDFCoefficients(gains.p, gains.i, gains.d, gains.f);
            back.setVelocityPIDFCoefficients(gains.p, gains.i, gains.d, gains.f);

            double ticksPerSecondPerRPM = PIDTuning.MOTOR_TICKS_PER_REV
                    / (60.0 * PIDTuning.FLYWHEEL_RATIO);
            double rpmPerTickPerSecond = 1.0 / ticksPerSecondPerRPM;
            telemetry.setMsTransmissionInterval(100);
            showControls();
            telemetry.addData("flyFront / flyBack target RPM", "%.0f / %.0f", frontTargetRPM, backTargetRPM);
            telemetry.addData("P / I / D / F from PIDTuning", "%.4f / %.4f / %.4f / %.4f",
                    gains.p, gains.i, gains.d, gains.f);
            telemetry.update();
            waitForStart();

            boolean previousUp = false, previousDown = false, previousY = false, previousX = false;
            while (opModeIsActive()) {
                boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
                boolean y = gamepad1.y, x = gamepad1.x;
                double frontChange = 0.0, backChange = 0.0;
                if (up && !previousUp) frontChange += RPM_STEP;
                if (down && !previousDown) frontChange -= RPM_STEP;
                if (y && !previousY) backChange += RPM_STEP;
                if (x && !previousX) backChange -= RPM_STEP;
                frontTargetRPM = adjustedRPM(frontTargetRPM, frontChange);
                backTargetRPM = adjustedRPM(backTargetRPM, backChange);
                previousUp = up;
                previousDown = down;
                previousY = y;
                previousX = x;

                boolean run = gamepad1.a && !gamepad1.b;
                if (run) {
                    front.setVelocity(frontTargetRPM * ticksPerSecondPerRPM);
                    back.setVelocity(backTargetRPM * ticksPerSecondPerRPM);
                } else {
                    front.setPower(0.0);
                    back.setPower(0.0);
                }

                showControls();
                telemetry.addData("Flywheels", run ? "RUNNING" : "STOPPED");
                telemetry.addData("flyFront target RPM", "%.0f", frontTargetRPM);
                telemetry.addData("flyBack target RPM", "%.0f", backTargetRPM);
                telemetry.addData("flyFront measured RPM", "%.1f", front.getVelocity() * rpmPerTickPerSecond);
                telemetry.addData("flyBack measured RPM", "%.1f", back.getVelocity() * rpmPerTickPerSecond);
                telemetry.addData("flyFront / flyBack encoder ticks", "%d / %d",
                        front.getCurrentPosition(), back.getCurrentPosition());
                telemetry.update();
                sleep(20);
            }
        } finally {
            front.setPower(0.0);
            back.setPower(0.0);
        }
    }

    private static double adjustedRPM(double targetRPM, double speedChange) {
        double maxRPM = PIDTuning.MOTOR_MAX_RPM * PIDTuning.FLYWHEEL_RATIO;
        double speed = Math.max(0.0, Math.min(maxRPM, Math.abs(targetRPM) + speedChange));
        // Preserve the motor's spin direction, including when slowed all the way to zero.
        return Math.copySign(speed, targetRPM);
    }

    private void showControls() {
        telemetry.addLine("Hold A: run both flywheels | Release A or B: stop");
        telemetry.addLine("flyFront: D-pad up faster / down slower | flyBack: Y faster / X slower");
        telemetry.addLine("100 RPM per press; keeps spin direction. Targets reset on INIT.");
    }

    private static void configureMotor(DcMotorEx motor, DcMotor.Direction direction) {
        motor.setPower(0.0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorConfigurationType type = motor.getMotorType().clone();
        type.setMaxRPM(PIDTuning.MOTOR_MAX_RPM);
        type.setTicksPerRev(PIDTuning.MOTOR_TICKS_PER_REV);
        type.setGearing(1.0);
        motor.setMotorType(type);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
