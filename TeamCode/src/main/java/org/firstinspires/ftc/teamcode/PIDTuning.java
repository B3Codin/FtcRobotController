package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**
 * Tunes the REV Hub's encoder velocity PIDF loop, not a separate setPower PID loop.
 * Add an encoder-equipped motor named "flywheel" to the robot configuration.
 * Motor specifications below are for the team's 6000 RPM, 1:1 goBILDA flywheel.
 * Overrides the generic series preset for this OpMode only.
 * Live changes are temporary; copy tuned values into the defaults below to save them.
 */
@Configurable
@TeleOp(name = "PIDTuning", group = "Tuning")
public class PIDTuning extends LinearOpMode {
    private static final String MOTOR_NAME = "flywheel";
    private static final DcMotor.Direction MOTOR_DIRECTION = DcMotor.Direction.FORWARD;
    // The generic SDK "GoBILDA 5201 series" preset is 104 RPM / 1500 ticks,
    // not the 6000 RPM, 1:1 motor used here.
    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    // Flywheel revolutions per configured motor-output-shaft revolution (1 for direct drive).
    private static final double FLYWHEEL_RATIO = 1.0;
    private static final double RPM_STEP = 100.0;
    private static final double RPM_TOLERANCE = 50.0;
    // REV velocity feedforward uses ticks/second and a full-output scale of 32767.
    private static final double STARTING_F = 32767.0 / (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REV / 60.0);
    private static final String[] SETTINGS = {"P", "I", "D", "F", "Target RPM"};
    private static final double[] COEFFICIENT_STEPS = {0.001, 0.01, 0.1, 1.0, 10.0};

    // Editable live in Panels, or with the gamepad controls shown in telemetry.
    // These are REV Hub velocity-loop coefficients, not power-per-RPM coefficients.
    public static volatile double P = 0.0;
    public static volatile double I = 0.0;
    public static volatile double D = 0.0;
    public static volatile double F = STARTING_F;
    public static volatile double TARGET_RPM = 0.0;

    private DcMotorEx flywheel;
    private PIDFCoefficients appliedPIDF;
    private boolean running;
    private int selectedSetting;
    private int stepIndex = 2;
    private boolean previousA, previousLeft, previousRight, previousUp, previousDown;
    private boolean previousLeftBumper, previousRightBumper, previousX;

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel = hardwareMap.tryGet(DcMotorEx.class, MOTOR_NAME);
        if (flywheel == null) {
            telemetry.addLine("Add a motor named '" + MOTOR_NAME + "' to the robot configuration first.");
            telemetry.addLine("Connect its encoder and select the correct motor type.");
            telemetry.update();
            waitForStart();
            return;
        }

        try {
            flywheel.setPower(0.0);
            flywheel.setDirection(MOTOR_DIRECTION);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            MotorConfigurationType configuredType = flywheel.getMotorType();
            // Clone before editing: SDK motor-type objects can be shared by other motors.
            MotorConfigurationType flywheelType = configuredType.clone();
            flywheelType.setMaxRPM(MOTOR_MAX_RPM);
            flywheelType.setTicksPerRev(MOTOR_TICKS_PER_REV);
            flywheelType.setGearing(1.0);
            flywheel.setMotorType(flywheelType);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double ticksPerRev = flywheel.getMotorType().getTicksPerRev();
            double maxRPM = flywheel.getMotorType().getMaxRPM() * FLYWHEEL_RATIO;
            if (!Double.isFinite(ticksPerRev) || ticksPerRev <= 0.0
                    || !Double.isFinite(maxRPM) || maxRPM <= 0.0 || FLYWHEEL_RATIO <= 0.0) {
                telemetry.addLine("Invalid motor configuration: check motor type and FLYWHEEL_RATIO.");
                telemetry.update();
                waitForStart();
                return;
            }

            telemetry.setMsTransmissionInterval(100);
            telemetry.addData("Robot configuration preset", configuredType.getName());
            telemetry.addData("Preset RPM / ticks per rev", "%.0f / %.0f",
                    configuredType.getMaxRPM(), configuredType.getTicksPerRev());
            telemetry.addData("Active motor RPM / ticks per rev", "%.0f / %.0f", MOTOR_MAX_RPM, ticksPerRev);
            telemetry.addData("Maximum flywheel RPM", "%.0f", maxRPM);
            telemetry.addData("Starting F estimate", "%.4f (tune under load)", STARTING_F);
            showControls();
            telemetry.addLine("Set a positive target RPM, then press A after START. Tune F first, then P.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                updateControls();
                // Snapshot values once per loop because Panels may edit them concurrently.
                double p = P, i = I, d = D, f = F, targetRPM = TARGET_RPM;
                boolean validPIDF = validCoefficient(p) && validCoefficient(i)
                        && validCoefficient(d) && validCoefficient(f);
                boolean validRPM = Double.isFinite(targetRPM) && targetRPM >= 0.0 && targetRPM <= maxRPM;
                boolean zeroPIDF = p == 0.0 && i == 0.0 && d == 0.0 && f == 0.0;
                boolean valid = validPIDF && validRPM && !zeroPIDF;
                String status;
                if (!valid) {
                    running = false;
                    flywheel.setPower(0.0);
                    if (!validPIDF) {
                        status = "STOPPED: PIDF must be finite and nonnegative; fix, then press A";
                    } else if (!validRPM) {
                        status = "STOPPED: target outside 0.." + maxRPM + " RPM; fix, then press A";
                    } else {
                        status = "STOPPED: all gains zero; set F near " + STARTING_F + ", then press A";
                    }
                } else {
                    if (appliedPIDF == null || p != appliedPIDF.p || i != appliedPIDF.i
                            || d != appliedPIDF.d || f != appliedPIDF.f) {
                        flywheel.setVelocityPIDFCoefficients(p, i, d, f);
                        appliedPIDF = new PIDFCoefficients(p, i, d, f);
                    }
                    if (running && targetRPM > 0.0) {
                        flywheel.setVelocity(targetRPM * ticksPerRev / (60.0 * FLYWHEEL_RATIO));
                        status = "RUNNING (velocity commanded)";
                    } else {
                        flywheel.setPower(0.0);
                        status = targetRPM == 0.0 ? "STOPPED: set a positive target RPM"
                                : "STOPPED: press A to run";
                    }
                }

                double measuredRPM = flywheel.getVelocity() * 60.0 * FLYWHEEL_RATIO / ticksPerRev;
                telemetry.addData("Flywheel", status);
                telemetry.addData("Target / measured RPM", "%.0f / %.0f", targetRPM, measuredRPM);
                telemetry.addData("RPM error", "%.1f", targetRPM - measuredRPM);
                telemetry.addData("At speed (+/- " + RPM_TOLERANCE + " RPM)",
                        running && valid && targetRPM > 0.0 && Math.abs(targetRPM - measuredRPM) <= RPM_TOLERANCE);
                telemetry.addData("P / I / D / F", "%.4f / %.4f / %.4f / %.4f", p, i, d, f);
                telemetry.addData("Selected setting", SETTINGS[selectedSetting]);
                telemetry.addData("Adjustment step", selectedSetting == 4 ? RPM_STEP : COEFFICIENT_STEPS[stepIndex]);
                telemetry.addData("Configured maximum flywheel RPM", "%.0f", maxRPM);
                telemetry.addData("Encoder ticks per motor rev", "%.0f", ticksPerRev);
                telemetry.addData("Encoder position / ticks per second", "%d / %.1f",
                        flywheel.getCurrentPosition(), flywheel.getVelocity());
                showControls();
                telemetry.update();
                sleep(20);
            }
        } finally {
            // Removing power lets the flywheel coast; B/STOP do not actively reverse it.
            flywheel.setPower(0.0);
        }
    }

    private void updateControls() {
        if (gamepad1.dpad_left && !previousLeft) selectedSetting = (selectedSetting + 4) % SETTINGS.length;
        if (gamepad1.dpad_right && !previousRight) selectedSetting = (selectedSetting + 1) % SETTINGS.length;
        if (gamepad1.left_bumper && !previousLeftBumper) stepIndex = Math.max(0, stepIndex - 1);
        if (gamepad1.right_bumper && !previousRightBumper) stepIndex = Math.min(COEFFICIENT_STEPS.length - 1, stepIndex + 1);
        double change = 0.0;
        double step = selectedSetting == 4 ? RPM_STEP : COEFFICIENT_STEPS[stepIndex];
        if (gamepad1.dpad_up && !previousUp) change += step;
        if (gamepad1.dpad_down && !previousDown) change -= step;
        boolean reset = gamepad1.x && !previousX;
        if (change != 0.0 || reset) {
            switch (selectedSetting) {
                case 0: P = adjustedValue(P, change, reset); break;
                case 1: I = adjustedValue(I, change, reset); break;
                case 2: D = adjustedValue(D, change, reset); break;
                case 3: F = adjustedValue(F, change, reset); break;
                case 4: TARGET_RPM = adjustedValue(TARGET_RPM, change, reset); break;
                default: break;
            }
        }
        if (gamepad1.a && !previousA) running = !running;
        if (gamepad1.b) running = false; // Stop always wins over start.
        previousA = gamepad1.a;
        previousLeft = gamepad1.dpad_left;
        previousRight = gamepad1.dpad_right;
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;
        previousLeftBumper = gamepad1.left_bumper;
        previousRightBumper = gamepad1.right_bumper;
        previousX = gamepad1.x;
    }

    private static double adjustedValue(double value, double change, boolean reset) {
        if (reset) return 0.0;
        return change == 0.0 ? value : Math.max(0.0, value + change);
    }

    private static boolean validCoefficient(double value) {
        return Double.isFinite(value) && value >= 0.0;
    }

    private void showControls() {
        telemetry.addLine("A: start/stop | B: stop | X: zero selected setting");
        telemetry.addLine("D-pad left/right: select P/I/D/F/RPM | Up/down: change value");
        telemetry.addLine("Bumpers: smaller/larger coefficient step | Panels: edit numeric values live");
    }
}
