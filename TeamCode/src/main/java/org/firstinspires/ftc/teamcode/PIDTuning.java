package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Tunes the REV Hub's encoder velocity PIDF loop, not a separate setPower PID loop.
 * Add an encoder-equipped motor named "flywheel" to the robot configuration.
 * Select its actual motor type so ticks/revolution and RPM are correct.
 * Live changes are temporary; copy tuned values into the defaults below to save them.
 */
@Configurable
@TeleOp(name = "PIDTuning", group = "Tuning")
public class PIDTuning extends LinearOpMode {
    private static final String MOTOR_NAME = "flywheel";
    private static final DcMotor.Direction MOTOR_DIRECTION = DcMotor.Direction.FORWARD;
    // Flywheel revolutions per configured motor-output-shaft revolution (1 for direct drive).
    private static final double FLYWHEEL_RATIO = 1.0;
    private static final double RPM_STEP = 100.0;
    private static final double RPM_TOLERANCE = 50.0;
    private static final String[] SETTINGS = {"P", "I", "D", "F", "Target RPM"};
    private static final double[] COEFFICIENT_STEPS = {0.001, 0.01, 0.1, 1.0, 10.0};

    // Editable live in Panels, or with the gamepad controls shown in telemetry.
    // These are REV Hub velocity-loop coefficients, not power-per-RPM coefficients.
    public static volatile double P = 0.0;
    public static volatile double I = 0.0;
    public static volatile double D = 0.0;
    public static volatile double F = 0.0;
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
            showControls();
            telemetry.addLine("All PIDF defaults are zero. Tune F first, then P; add I/D only as needed.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                updateControls();
                // Snapshot values once per loop because Panels may edit them concurrently.
                double p = P, i = I, d = D, f = F, targetRPM = TARGET_RPM;
                boolean valid = validCoefficient(p) && validCoefficient(i)
                        && validCoefficient(d) && validCoefficient(f)
                        && Double.isFinite(targetRPM) && targetRPM >= 0.0 && targetRPM <= maxRPM;
                if (!valid) {
                    running = false;
                    flywheel.setPower(0.0);
                    telemetry.addLine("Stopped: use finite, nonnegative PIDF values and target RPM within range.");
                } else {
                    if (appliedPIDF == null || p != appliedPIDF.p || i != appliedPIDF.i
                            || d != appliedPIDF.d || f != appliedPIDF.f) {
                        flywheel.setVelocityPIDFCoefficients(p, i, d, f);
                        appliedPIDF = new PIDFCoefficients(p, i, d, f);
                    }
                    if (running && targetRPM > 0.0) {
                        flywheel.setVelocity(targetRPM * ticksPerRev / (60.0 * FLYWHEEL_RATIO));
                    } else {
                        flywheel.setPower(0.0);
                    }
                }

                double measuredRPM = flywheel.getVelocity() * 60.0 * FLYWHEEL_RATIO / ticksPerRev;
                telemetry.addData("Flywheel", running && valid && targetRPM > 0.0 ? "RUNNING" : "STOPPED");
                telemetry.addData("Target / measured RPM", "%.0f / %.0f", targetRPM, measuredRPM);
                telemetry.addData("RPM error", "%.1f", targetRPM - measuredRPM);
                telemetry.addData("At speed (+/- " + RPM_TOLERANCE + " RPM)",
                        running && valid && targetRPM > 0.0 && Math.abs(targetRPM - measuredRPM) <= RPM_TOLERANCE);
                telemetry.addData("P / I / D / F", "%.4f / %.4f / %.4f / %.4f", p, i, d, f);
                telemetry.addData("Selected setting", SETTINGS[selectedSetting]);
                telemetry.addData("Adjustment step", selectedSetting == 4 ? RPM_STEP : COEFFICIENT_STEPS[stepIndex]);
                telemetry.addData("Configured maximum flywheel RPM", "%.0f", maxRPM);
                if (p == 0.0 && i == 0.0 && d == 0.0 && f == 0.0) {
                    telemetry.addLine("PIDF is all zero: no spin-up output until you tune the coefficients.");
                }
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
