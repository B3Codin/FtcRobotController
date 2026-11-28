package org.firstinspires.ftc.teamcode;

/** Startup and encoder tracking without a magnetic reference. */
public final class JackbotsManualZeroTest {
    private static int checks;
    private static final TurretAimController.Settings SETTINGS =
            new TurretAimController.Settings(0.005, 0, 0, 0.08, 0.1, 1, 0.2, 1, 360, 2);

    private static void expect(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static double step(JackbotsTeleOp.TurretTracking turret, int ticks, double now,
                               boolean zero, boolean toggle, boolean right) {
        return turret.update(ticks, now, true, 30, zero, toggle, false, right, SETTINGS);
    }

    public static void main(String[] args) {
        JackbotsTeleOp.TurretTracking turret = new JackbotsTeleOp.TurretTracking(12345, false);
        turret.initSample(12400, false);
        expect(step(turret, 12400, 0, false, false, true) == 0, "INIT never moves turret");
        turret.onStart(false, false);
        expect(step(turret, 12400, 0.1, false, false, false) == 0, "START does not home turret");
        expect(step(turret, 12400, 20, false, false, false) == 0, "No homing timeout or automatic motion");
        expect(turret.state().contains("D-pad Down"), "Driver sees manual zero instruction");
        step(turret, 12400, 20.1, false, true, false);
        step(turret, 12400, 20.2, false, false, false);
        expect(step(turret, 12400, 20.3, false, true, false) == 0, "Auto toggle cannot bypass zero");
        expect(step(turret, 12400, 20.4, false, false, true) > 0, "Manual control works before zero");
        expect(step(turret, 12500, 20.5, true, false, false) == 0, "Manual zero stops motion");
        expect(turret.angleDegrees() == 0, "Manual zero uses current encoder count");
        expect(step(turret, 12500, 20.6, false, false, false) == 0, "Manual override leaves auto disabled");
        expect(step(turret, 12500, 20.7, false, true, false) > 0, "Auto resumes with explicit toggle");
        step(turret, 13353, 20.8, false, false, false);
        expect(Math.abs(turret.angleDegrees() - 10) < 0.01, "Runtime angle follows encoder from manual zero");
        expect(step(turret, 13353, 20.9, true, false, false) == 0, "Runtime recalibration stops auto");
        expect(turret.angleDegrees() == 0, "Runtime recalibration sets new zero");
        step(turret, 13400, 21, true, false, false);
        expect(turret.angleDegrees() > 0, "Holding zero does not repeatedly change reference");

        turret = new JackbotsTeleOp.TurretTracking(12345, true);
        turret.initSample(12400, true);
        turret.onStart(true, false);
        step(turret, 12400, 0, false, false, false);
        expect(turret.state().contains("D-pad Down"), "Zero held at INIT requires a fresh press");

        turret = new JackbotsTeleOp.TurretTracking(12345, false);
        turret.initSample(12400, true);
        turret.initSample(13253, false);
        turret.onStart(false, false);
        expect(Math.abs(turret.angleDegrees() - 10) < 0.01, "START preserves INIT zero and subsequent movement");
        expect(step(turret, 13253, 0, false, false, false) > 0, "Calibrated START aims directly without homing");
        System.out.println("Passed " + checks + " manual zero checks.");
    }
}
