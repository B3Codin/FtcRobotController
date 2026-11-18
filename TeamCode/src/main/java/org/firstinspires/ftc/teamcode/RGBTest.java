package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/** Configure two separate servo ports as LeftRGB and RightRGB. Uses gamepad 1 only. */
@TeleOp(name = "RGBTest", group = "Test")
public class RGBTest extends LinearOpMode {
    private static final int[] STEPS = {1, 5, 10, 25, 50};
    private static final String[] CHANNELS = {"Red", "Green", "Blue"};
    private static final String[] TARGETS = {"LeftRGB", "RightRGB", "Both"};
    private static final String[] PRESET_NAMES = {
            "Red", "Orange", "Yellow", "Green", "Blue", "Indigo", "Violet", "White", "Off"
    };
    private static final int[][] PRESETS = {
            {255, 0, 0}, {255, 128, 0}, {255, 255, 0}, {0, 255, 0},
            {0, 0, 255}, {75, 0, 130}, {148, 0, 211}, {255, 255, 255}, {0, 0, 0}
    };

    // goBILDA 3118-0808-0002 Product Insight #4, expressed in microseconds.
    // Explicitly using 500-2500 us avoids relying on the SDK's default PWM range.
    private static final double PWM_MIN_US = 1300;
    private static final double PWM_MAX_US = 2500;
    private static final double OFF_US = 500;
    private static final double WHITE_US = 2500;
    private static final double[] HUES = {0, 30.118, 60, 90, 120, 210, 240, 274.615, 282.085};
    private static final double[] PULSES = {1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900};

    @Override
    public void runOpMode() {
        ServoImplEx[] indicators = {
                hardwareMap.get(ServoImplEx.class, "LeftRGB"),
                hardwareMap.get(ServoImplEx.class, "RightRGB")
        };
        int[][] rgb = {{0, 0, 0}, {0, 0, 0}};
        String[] modes = {"Off", "Off"};
        int target = 2;
        int channel = 0;
        int stepIndex = 1;
        int presetIndex = PRESETS.length - 1;
        Gamepad previous = new Gamepad();
        Gamepad current = new Gamepad();

        try {
            for (ServoImplEx indicator : indicators) {
                indicator.setDirection(Servo.Direction.FORWARD);
                indicator.setPwmRange(new PwmControl.PwmRange(PWM_MIN_US, PWM_MAX_US));
                indicator.setPosition(positionForPulse(OFF_US));
            }
            addControls();
            telemetry.addLine("Ready: both indicators start off.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;
            previous.copy(gamepad1);

            while (opModeIsActive()) {
                current.copy(gamepad1);
                if (current.a && !previous.a) target = (target + 1) % TARGETS.length;
                if (current.x && !previous.x) channel = (channel + 1) % CHANNELS.length;
                if (current.y && !previous.y) stepIndex = (stepIndex + 1) % STEPS.length;

                boolean next = current.right_bumper && !previous.right_bumper;
                boolean back = current.left_bumper && !previous.left_bumper;
                if (next != back) {
                    presetIndex = (presetIndex + (next ? 1 : -1) + PRESETS.length) % PRESETS.length;
                    for (int i = 0; i < indicators.length; i++) {
                        if (target != 2 && target != i) continue;
                        System.arraycopy(PRESETS[presetIndex], 0, rgb[i], 0, 3);
                        modes[i] = PRESET_NAMES[presetIndex];
                    }
                }

                boolean up = current.dpad_up;
                boolean down = current.dpad_down;
                if (up != down && ((up && !previous.dpad_up) || (down && !previous.dpad_down))) {
                    int delta = up ? STEPS[stepIndex] : -STEPS[stepIndex];
                    for (int i = 0; i < indicators.length; i++) {
                        if (target != 2 && target != i) continue;
                        rgb[i][channel] = Math.max(0, Math.min(255, rgb[i][channel] + delta));
                        modes[i] = "Custom";
                    }
                }

                telemetry.addData("Selected indicator(s)", TARGETS[target]);
                telemetry.addData("Selected channel", CHANNELS[channel]);
                telemetry.addData("Step", STEPS[stepIndex]);
                telemetry.addData("Preset cursor", PRESET_NAMES[presetIndex]);
                for (int i = 0; i < indicators.length; i++) {
                    double pulse = pulseForRgb(rgb[i]);
                    indicators[i].setPosition(positionForPulse(pulse));
                    telemetry.addData(TARGETS[i], "%s | R %d G %d B %d | %.0f us",
                            modes[i], rgb[i][0], rgb[i][1], rgb[i][2], pulse);
                }
                addControls();
                telemetry.update();
                previous.copy(current);
                sleep(20);
            }
        } finally {
            for (ServoImplEx indicator : indicators) {
                indicator.setPosition(positionForPulse(OFF_US));
            }
        }
    }

    private void addControls() {
        telemetry.addLine("Gamepad 1: A = Left / Right / Both | X = R / G / B");
        telemetry.addLine("Y = step 1 / 5 / 10 / 25 / 50 | D-pad up/down = +/- channel");
        telemetry.addLine("LB/RB = previous/next preset (rainbow, white, off)");
        telemetry.addLine("One change per press. Each indicator keeps its own RGB settings.");
        telemetry.addLine("PWM output approximates hue; brightness/saturation are fixed by hardware.");
    }

    private static double positionForPulse(double pulseUs) {
        return (pulseUs - PWM_MIN_US) / (PWM_MAX_US - PWM_MIN_US);
    }

    private static double pulseForRgb(int[] rgb) {
        int max = Math.max(rgb[0], Math.max(rgb[1], rgb[2]));
        int min = Math.min(rgb[0], Math.min(rgb[1], rgb[2]));
        if (max == 0) return OFF_US;
        if (max == min) return WHITE_US;

        // This indicator accepts one hue command, not three independent RGB channels.
        // Approximate the requested hue using the manufacturer's color landmarks.
        float[] hsv = new float[3];
        Color.RGBToHSV(rgb[0], rgb[1], rgb[2], hsv);
        double hue = hsv[0];
        int last = HUES.length - 1;
        if (hue > HUES[last]) {
            // Unsupported hues between violet and red use the nearer endpoint.
            return (360 - hue < hue - HUES[last]) ? PULSES[0] : PULSES[last];
        }
        for (int i = 1; i < HUES.length; i++) {
            if (hue <= HUES[i]) {
                double fraction = (hue - HUES[i - 1]) / (HUES[i] - HUES[i - 1]);
                return PULSES[i - 1] + fraction * (PULSES[i] - PULSES[i - 1]);
            }
        }
        return PULSES[last];
    }
}
