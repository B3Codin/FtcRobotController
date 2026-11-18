package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

/** Configure an I2C device of type REV Color Sensor V3 named colorSensor. */
@TeleOp(name = "ColorSenseTest", group = "Test")
public class ColorSenseTest extends LinearOpMode {
    // ---- Change these settings to calibrate for your balls and lighting. ----
    private static final String SENSOR_NAME = "colorSensor";
    private static final float SENSOR_GAIN = 2.0f;
    private static final float MIN_SIGNAL = 0.005f; // Strongest normalized channel, 0-1.
    private static final float MIN_SATURATION = 0.20f; // Reject white/gray, 0-1.

    // RGB thresholds use the peak-scaled 0-255 values shown in telemetry, NOT raw counts.
    // Each reading is scaled so its strongest channel is 255; channel ratios are preserved.
    private static final int RED_MIN_RED = 200;
    private static final int RED_MAX_GREEN = 80;
    private static final int RED_MAX_BLUE = 80;
    private static final int BLUE_MIN_BLUE = 200;
    private static final int BLUE_MAX_RED = 50;
    private static final int BLUE_MAX_GREEN = 50;
    private static final int YELLOW_MIN_RED = 180;
    private static final int YELLOW_MIN_GREEN = 180;
    private static final int YELLOW_MAX_BLUE = 100;

    // HSV hue is ONE angle, 0-360 degrees, separate from the three RGB channel values.
    // Choose the nearest target hue when a strict RGB threshold does not match.
    private static final float RED_HUE = 0;
    private static final float YELLOW_HUE = 60;
    private static final float BLUE_HUE = 240;
    private static final float MAX_HUE_ERROR = 45; // Farther away => UNKNOWN.
    // ---- End of calibration settings. ----

    @Override
    public void runOpMode() {
        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, SENSOR_NAME);
        sensor.setGain(SENSOR_GAIN);
        float[] hsv = new float[3];

        telemetry.addData("Ready", "REV Color Sensor V3: %s", SENSOR_NAME);
        telemetry.addLine("Set the sensor's physical LED switch on for consistent close-range tests.");
        telemetry.addLine("RGB thresholds and hue targets are at the top of ColorSenseTest.java.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            float peak = Math.max(colors.red, Math.max(colors.green, colors.blue));
            int red = scaleChannel(colors.red, peak);
            int green = scaleChannel(colors.green, peak);
            int blue = scaleChannel(colors.blue, peak);
            // Scaling before conversion preserves hue even when the sensor signal is very small.
            Color.RGBToHSV(red, green, blue, hsv);

            String thresholdMatch = classifyRgb(red, green, blue);
            String closestHue = closestHue(hsv[0]);
            String classification;
            String method;
            if (peak < MIN_SIGNAL || peak <= 0 || hsv[1] < MIN_SATURATION) {
                classification = "UNKNOWN";
                method = "Too dark or too little saturation";
            } else if (!thresholdMatch.equals("UNKNOWN")) {
                classification = thresholdMatch;
                method = "RGB thresholds";
            } else {
                classification = closestHue;
                method = "Nearest hue within MAX_HUE_ERROR";
            }

            telemetry.addData("Ball color", classification);
            telemetry.addData("Classification method", method);
            telemetry.addData("RGB (0-255, peak-scaled)", "R %d | G %d | B %d", red, green, blue);
            telemetry.addData("Normalized RGB (0-1)", "R %.5f | G %.5f | B %.5f",
                    colors.red, colors.green, colors.blue);
            telemetry.addData("Raw sensor RGB counts", "R %d | G %d | B %d",
                    sensor.red(), sensor.green(), sensor.blue());
            telemetry.addData("Hue (degrees)", "%.1f", hsv[0]);
            telemetry.addData("Saturation (0-1)", "%.3f", hsv[1]);
            telemetry.addData("Brightness (normalized peak)", "%.5f", peak);
            telemetry.addData("RGB threshold match (before signal check)", thresholdMatch);
            telemetry.addData("Closest hue (before signal check)", closestHue);
            telemetry.addData("Hue error: Red / Yellow / Blue", "%.1f / %.1f / %.1f deg",
                    hueDistance(hsv[0], RED_HUE), hueDistance(hsv[0], YELLOW_HUE),
                    hueDistance(hsv[0], BLUE_HUE));
            telemetry.addData("Gain", SENSOR_GAIN);
            telemetry.update();
            sleep(50);
        }
    }

    private static int scaleChannel(float channel, float peak) {
        return peak <= 0 ? 0 : Math.max(0, Math.min(255, Math.round(channel / peak * 255)));
    }

    private static String classifyRgb(int red, int green, int blue) {
        if (blue >= BLUE_MIN_BLUE && red <= BLUE_MAX_RED && green <= BLUE_MAX_GREEN) return "BLUE";
        if (red >= RED_MIN_RED && green <= RED_MAX_GREEN && blue <= RED_MAX_BLUE) return "RED";
        if (red >= YELLOW_MIN_RED && green >= YELLOW_MIN_GREEN && blue <= YELLOW_MAX_BLUE) return "YELLOW";
        return "UNKNOWN";
    }

    private static String closestHue(float hue) {
        float bestError = hueDistance(hue, RED_HUE);
        String best = "RED";
        float yellowError = hueDistance(hue, YELLOW_HUE);
        if (yellowError < bestError) {
            bestError = yellowError;
            best = "YELLOW";
        }
        float blueError = hueDistance(hue, BLUE_HUE);
        if (blueError < bestError) {
            bestError = blueError;
            best = "BLUE";
        }
        return bestError <= MAX_HUE_ERROR ? best : "UNKNOWN";
    }

    private static float hueDistance(float first, float second) {
        float difference = Math.abs(first - second) % 360;
        return Math.min(difference, 360 - difference);
    }
}
