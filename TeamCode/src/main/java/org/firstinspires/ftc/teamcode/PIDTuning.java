package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/**
 * Tunes proportional feedback (P) and feedforward (F) in each flywheel's REV Hub
 * encoder velocity loop. Integral and derivative gains are always zero.
 * Live changes are temporary; copy tuned gains into the defaults here and the
 * matching per-motor constants in JackbotsTeleOp to save them.
 */
@Configurable
@TeleOp(name = "PIDTuning", group = "Tuning")
public class PIDTuning extends LinearOpMode {
    // Shared with flywheelTest: 28 ticks/rev, direct-drive goBILDA motors.
    // Override the generic 104 RPM / 1500 ticks-per-rev series preset.
    static final double FRONT_MOTOR_MAX_RPM = 1150.0;
    static final double BACK_MOTOR_MAX_RPM = 6000.0;
    static final double MOTOR_TICKS_PER_REV = 28.0;
    static final double FLYWHEEL_RATIO = 1.0;
    private static final double RPM_STEP = 100.0;
    private static final double RPM_TOLERANCE = 50.0;
    private static final String[] MOTOR_NAMES = {"flyFront", "flyBack"};
    private static final String[] SETTINGS = {"P", "F", "Target RPM"};
    private static final double[] COEFFICIENT_STEPS = {0.001, 0.01, 0.1, 1.0, 10.0};

    // Edit live in Panels or with gamepad 1. These are REV velocity-loop gains.
    public static volatile double FRONT_P = 0.0;
    public static volatile double FRONT_F = startingF(FRONT_MOTOR_MAX_RPM);
    public static volatile double FRONT_TARGET_RPM = -FRONT_MOTOR_MAX_RPM * FLYWHEEL_RATIO;
    public static volatile double BACK_P = 0.0;
    public static volatile double BACK_F = startingF(BACK_MOTOR_MAX_RPM);
    public static volatile double BACK_TARGET_RPM = -2100.0;

    private final boolean[] running = new boolean[2];
    private final PIDFCoefficients[] appliedPIDF = new PIDFCoefficients[2];
    private int selectedMotor;
    private int selectedSetting;
    private int stepIndex = 2;
    private boolean previousA, previousX, previousY;
    private boolean previousLeft, previousRight, previousUp, previousDown;
    private boolean previousLeftBumper, previousRightBumper;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx[] motors = {
                hardwareMap.tryGet(DcMotorEx.class, MOTOR_NAMES[0]),
                hardwareMap.tryGet(DcMotorEx.class, MOTOR_NAMES[1])
        };

        try {
            for (DcMotorEx motor : motors) {
                if (motor != null) motor.setPower(0.0);
            }
            if (motors[0] == null || motors[1] == null) {
                telemetry.addLine("Configure encoder-equipped motors named flyFront and flyBack.");
                telemetry.addLine("Connect both encoders and select the correct motor types.");
                telemetry.update();
                waitForStart();
                return;
            }

            configureMotor(motors[0], DcMotor.Direction.FORWARD, FRONT_MOTOR_MAX_RPM);
            configureMotor(motors[1], DcMotor.Direction.FORWARD, BACK_MOTOR_MAX_RPM);
            double[] motorMaxRPM = {FRONT_MOTOR_MAX_RPM, BACK_MOTOR_MAX_RPM};
            double ticksPerSecondPerRPM = MOTOR_TICKS_PER_REV / (60.0 * FLYWHEEL_RATIO);

            telemetry.setMsTransmissionInterval(100);
            telemetry.addData("flyFront / flyBack maximum RPM", "%.0f / %.0f",
                    FRONT_MOTOR_MAX_RPM * FLYWHEEL_RATIO, BACK_MOTOR_MAX_RPM * FLYWHEEL_RATIO);
            telemetry.addData("Motor ticks per revolution", "%.0f", MOTOR_TICKS_PER_REV);
            telemetry.addData("flyFront / flyBack starting F", "%.4f / %.4f (tune under load)",
                    startingF(FRONT_MOTOR_MAX_RPM), startingF(BACK_MOTOR_MAX_RPM));
            telemetry.addLine("Both motors start OFF. Tune F first, then P, for each motor.");
            telemetry.addLine("Negative RPM matches JackbotsTeleOp; use Panels to change sign.");
            showControls();
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                updateControls();
                // Snapshot each live value once per loop; Panels can edit them concurrently.
                double[] p = {FRONT_P, BACK_P};
                double[] f = {FRONT_F, BACK_F};
                double[] targetRPM = {FRONT_TARGET_RPM, BACK_TARGET_RPM};

                telemetry.addData("Selected motor", MOTOR_NAMES[selectedMotor]);
                telemetry.addData("Selected setting", SETTINGS[selectedSetting]);
                telemetry.addData("Adjustment step", selectedSetting == 2
                        ? RPM_STEP : COEFFICIENT_STEPS[stepIndex]);

                for (int index = 0; index < motors.length; index++) {
                    DcMotorEx motor = motors[index];
                    String problem = settingsProblem(p[index], f[index], targetRPM[index], motorMaxRPM[index]);
                    String status;
                    if (problem != null) {
                        running[index] = false;
                        motor.setPower(0.0);
                        status = "STOPPED: " + problem + "; fix, then press A";
                    } else {
                        PIDFCoefficients applied = appliedPIDF[index];
                        if (applied == null || p[index] != applied.p || f[index] != applied.f) {
                            motor.setVelocityPIDFCoefficients(p[index], 0.0, 0.0, f[index]);
                            appliedPIDF[index] = new PIDFCoefficients(p[index], 0.0, 0.0, f[index]);
                        }
                        if (running[index] && targetRPM[index] != 0.0) {
                            motor.setVelocity(targetRPM[index] * ticksPerSecondPerRPM);
                            status = "RUNNING";
                        } else {
                            motor.setPower(0.0);
                            status = targetRPM[index] == 0.0
                                    ? "STOPPED: target RPM is zero" : "STOPPED";
                        }
                    }

                    double measuredRPM = motor.getVelocity() / ticksPerSecondPerRPM;
                    String name = MOTOR_NAMES[index];
                    telemetry.addData(name + " status", status);
                    telemetry.addData(name + " maximum RPM", "%.0f", motorMaxRPM[index] * FLYWHEEL_RATIO);
                    telemetry.addData(name + " target / measured RPM", "%.0f / %.1f",
                            targetRPM[index], measuredRPM);
                    telemetry.addData(name + " RPM error", "%.1f", targetRPM[index] - measuredRPM);
                    telemetry.addData(name + " at speed (+/- " + RPM_TOLERANCE + " RPM)",
                            running[index] && problem == null && targetRPM[index] != 0.0
                                    && Math.abs(targetRPM[index] - measuredRPM) <= RPM_TOLERANCE);
                    telemetry.addData(name + " P / F", "%.4f / %.4f", p[index], f[index]);
                    telemetry.addData(name + " encoder ticks / ticks per second", "%d / %.1f",
                            motor.getCurrentPosition(), motor.getVelocity());
                }

                showControls();
                telemetry.update();
                sleep(20);
            }
        } finally {
            // B/STOP removes power from both flywheels and lets them coast.
            for (DcMotorEx motor : motors) {
                if (motor != null) motor.setPower(0.0);
            }
        }
    }

    private void updateControls() {
        boolean a = gamepad1.a, x = gamepad1.x, y = gamepad1.y;
        boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean leftBumper = gamepad1.left_bumper, rightBumper = gamepad1.right_bumper;

        if (y && !previousY) selectedMotor = 1 - selectedMotor;
        if (left && !previousLeft) selectedSetting = (selectedSetting + SETTINGS.length - 1) % SETTINGS.length;
        if (right && !previousRight) selectedSetting = (selectedSetting + 1) % SETTINGS.length;
        if (leftBumper && !previousLeftBumper) stepIndex = Math.max(0, stepIndex - 1);
        if (rightBumper && !previousRightBumper) stepIndex = Math.min(COEFFICIENT_STEPS.length - 1, stepIndex + 1);

        double change = 0.0;
        double step = selectedSetting == 2 ? RPM_STEP : COEFFICIENT_STEPS[stepIndex];
        if (up && !previousUp) change += step;
        if (down && !previousDown) change -= step;
        boolean reset = x && !previousX;
        if (change != 0.0 || reset) {
            if (selectedMotor == 0) {
                switch (selectedSetting) {
                    case 0: FRONT_P = adjustedCoefficient(FRONT_P, change, reset); break;
                    case 1: FRONT_F = adjustedCoefficient(FRONT_F, change, reset); break;
                    case 2:
                        FRONT_TARGET_RPM = adjustedRPM(FRONT_TARGET_RPM, change, reset,
                                FRONT_MOTOR_MAX_RPM * FLYWHEEL_RATIO);
                        break;
                    default: break;
                }
            } else {
                switch (selectedSetting) {
                    case 0: BACK_P = adjustedCoefficient(BACK_P, change, reset); break;
                    case 1: BACK_F = adjustedCoefficient(BACK_F, change, reset); break;
                    case 2:
                        BACK_TARGET_RPM = adjustedRPM(BACK_TARGET_RPM, change, reset,
                                BACK_MOTOR_MAX_RPM * FLYWHEEL_RATIO);
                        break;
                    default: break;
                }
            }
        }
        if (a && !previousA) running[selectedMotor] = !running[selectedMotor];
        if (gamepad1.b) {
            running[0] = false;
            running[1] = false;
        }
        previousA = a;
        previousX = x;
        previousY = y;
        previousLeft = left;
        previousRight = right;
        previousUp = up;
        previousDown = down;
        previousLeftBumper = leftBumper;
        previousRightBumper = rightBumper;
    }

    private static double adjustedCoefficient(double value, double change, boolean reset) {
        if (reset) return 0.0;
        return change == 0.0 ? value : Math.max(0.0, value + change);
    }

    private static double adjustedRPM(double value, double change, boolean reset, double maxRPM) {
        // Up/down adjusts speed magnitude, keeping the motor's commanded direction.
        // Use negative direction when starting from zero, matching the robot's setpoints.
        double direction = value == 0.0 ? -1.0 : value;
        if (reset) return Math.copySign(0.0, direction);
        double speed = Math.max(0.0, Math.min(maxRPM, Math.abs(value) + change));
        return Math.copySign(speed, direction);
    }

    private static String settingsProblem(double p, double f, double targetRPM, double motorMaxRPM) {
        if (!Double.isFinite(p) || p < 0.0 || !Double.isFinite(f) || f < 0.0) {
            return "P and F must be finite and nonnegative";
        }
        double maxRPM = motorMaxRPM * FLYWHEEL_RATIO;
        if (!Double.isFinite(targetRPM) || Math.abs(targetRPM) > maxRPM) {
            return "target outside +/- " + maxRPM + " RPM";
        }
        if (p == 0.0 && f == 0.0) {
            return "P and F are both zero; set F near " + startingF(motorMaxRPM);
        }
        return null;
    }

    private static double startingF(double motorMaxRPM) {
        // REV velocity feedforward uses ticks/second and a full-output scale of 32767.
        return 32767.0 / (motorMaxRPM * MOTOR_TICKS_PER_REV / 60.0);
    }

    static void configureMotor(DcMotorEx motor, DcMotor.Direction direction, double motorMaxRPM) {
        motor.setPower(0.0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Clone first: SDK motor-type objects can be shared by other motors.
        MotorConfigurationType type = motor.getMotorType().clone();
        type.setMaxRPM(motorMaxRPM);
        type.setTicksPerRev(MOTOR_TICKS_PER_REV);
        type.setGearing(1.0);
        motor.setMotorType(type);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void showControls() {
        telemetry.addLine("Y: select flyFront/flyBack | A: toggle selected motor | B: stop BOTH");
        telemetry.addLine("D-pad left/right: select P/F/RPM | Up/down: gain or speed magnitude +/-");
        telemetry.addLine("X: zero selected setting | Bumpers: smaller/larger coefficient step");
        telemetry.addLine("Panels: edit FRONT_P/F/TARGET_RPM and BACK_P/F/TARGET_RPM live");
    }
}
