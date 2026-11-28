package org.firstinspires.ftc.teamcode;

/** Field-direction presets and continuous heading compensation without robot hardware. */
public final class TurretFieldHeadingTest {
    private static int checks;
    private static final TurretAimController.Settings SETTINGS =
            new TurretAimController.Settings(0.005, 0, 0, 0.08, 0.1, 1, 0.2, 1, 360, 2);

    private static void expect(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static double step(TurretAimController controller, TurretAimTest.FieldHeadingSelection aim,
                               double angle, double heading, double now) {
        return controller.update(angle, Math.round(angle * TurretMagnetZero.TICKS_PER_DEGREE), now,
                false, false, false, false, false, SETTINGS, aim.isSelected(), aim.robotTargetDegrees(heading));
    }

    public static void main(String[] args) {
        for (int preset : new int[] {-90, 0, 90}) {
            TurretAimTest.FieldHeadingSelection aim = new TurretAimTest.FieldHeadingSelection();
            expect(aim.update(preset == 90, preset == 0, preset == -90), "Preset starts on a fresh press");
            expect(aim.robotTargetDegrees(0) == preset, "Initial heading preserves preset signs");
            for (double heading : new double[] {-450, -180, -90, -30, 0, 30, 90, 180, 450}) {
                double target = aim.robotTargetDegrees(heading);
                double fieldAngle = Math.toRadians(target - heading);
                expect(target >= -180 && target < 180, "Target stays inside turret travel limits");
                expect(Math.abs(Math.sin(fieldAngle) - Math.sin(Math.toRadians(preset))) < 1e-9
                                && Math.abs(Math.cos(fieldAngle) - Math.cos(Math.toRadians(preset))) < 1e-9,
                        "Robot target maintains the selected field direction across chassis rotation");
            }
        }

        TurretAimTest.FieldHeadingSelection aim = new TurretAimTest.FieldHeadingSelection();
        aim.onStart(true, false, false);
        expect(!aim.update(true, false, false) && !aim.isSelected(), "A button held at START cannot command motion");
        aim.update(false, false, false);
        expect(aim.update(true, false, false), "Release/re-press selects the preset");
        expect(!aim.update(true, false, false), "Holding a preset cannot repeatedly retry faults");
        aim.clear();
        expect(!aim.update(true, false, false) && !aim.isSelected(), "Manual cancellation survives a held button");
        aim.update(false, false, false);
        aim.update(true, true, true);
        expect(aim.robotTargetDegrees(0) == 0, "Center takes priority for simultaneous presses");
        aim.update(false, false, false);
        expect(aim.isSelected(), "Releasing a preset keeps the field direction selected");

        TurretAimController controller = new TurretAimController(false, false, false);
        expect(step(controller, aim, 0, 0, 0) == 0, "Field zero is initially aligned");
        expect(step(controller, aim, 0, 30, 0.05) > 0, "CCW chassis turn commands clockwise turret compensation");
        expect(step(controller, aim, 30, 30, 0.10) == 0, "Compensated angle stops at the field direction");
        expect(step(controller, aim, 30, 20, 0.15) < 0, "Direction hold resumes after alignment when chassis turns back");
        expect(step(controller, aim, 20, Double.NaN, 0.20) == 0, "Invalid heading stops the turret");
        expect(step(controller, aim, 20, 30, 0.25) == 0, "Restored heading cannot silently clear a fault");
        expect(aim.update(false, true, false), "A fresh preset press requests a retry");
        controller.cancel(20, Math.round(20 * TurretMagnetZero.TICKS_PER_DEGREE), 0.30);
        expect(step(controller, aim, 20, 30, 0.30) > 0, "Explicit retry resumes field hold");
        expect(step(controller, aim, 20, 30, 1.40) == 0, "Frozen encoder still produces a stall fault");

        controller = new TurretAimController(false, false, false);
        expect(step(controller, aim, 179, 179, 0) == 0, "Near the rear boundary the direction can be aligned");
        expect(step(controller, aim, 179, 181, 0.05) == 0
                        && controller.getState().contains("rear travel"),
                "Crossing the rear boundary stops rather than commanding a full revolution");
        expect(Double.isNaN(aim.robotTargetDegrees(Double.POSITIVE_INFINITY)), "Infinite heading is invalid");
        aim.clear();
        expect(!aim.isSelected() && Double.isNaN(aim.robotTargetDegrees(0)), "Clearing aim removes the field target");
        System.out.println("Passed " + checks + " field heading checks.");
    }
}
