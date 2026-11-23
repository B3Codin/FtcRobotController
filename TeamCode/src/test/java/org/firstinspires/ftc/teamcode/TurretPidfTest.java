package org.firstinspires.ftc.teamcode;

/** Standalone PIDF, live-setting, and power-limit regressions. */
public final class TurretPidfTest {
    private static int checks;

    private static void expect(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void near(double actual, double expected, String message) {
        expect(Math.abs(actual - expected) < 1e-9, message + ": " + actual);
    }

    private static TurretAimController.Settings settings(double p, double i, double d, double f,
                                                         double min, double max) {
        return new TurretAimController.Settings(p, i, d, f, min, max, 0.2, max, 360);
    }

    private static final class Rig {
        final TurretAimController controller = new TurretAimController(false, false, false);
        TurretAimController.Settings settings = settings(0.01, 0, 0, 0, 0.2, 0.5);
        double now;

        double update(double angle, double target, boolean tracking, boolean lb, boolean rb) {
            now += 0.05;
            return controller.update(angle, Math.round(angle * 8192 * 3.75 / 360), now,
                    lb, rb, false, false, false, settings, tracking, target);
        }

        double track(double angle, double target) {
            return update(angle, target, true, false, false);
        }
    }

    public static void main(String[] args) {
        Rig r = new Rig();
        near(r.track(0, 3), 0.2, "Minimum power lifts a small positive command");
        near(r.track(0, -3), -0.2, "Minimum power preserves negative direction");
        near(r.track(0, 100), 0.5, "Maximum caps a large command");
        near(r.track(100, 100), 0, "Minimum cannot keep turret moving at target");
        near(r.update(100, 100, false, false, false), 0, "Minimum cannot move an idle turret");
        near(r.update(100, 100, false, true, true), 0, "Minimum cannot override cancellation");
        near(r.update(180, 0, false, true, false), 0, "Minimum cannot override travel limit");
        near(r.update(180, 0, false, false, true), -0.2, "Inward manual recovery remains available");

        r = new Rig();
        near(r.track(0, 20), 0.2, "Initial gain applies");
        r.settings = settings(0.02, 0, 0, 0, 0, 0.5);
        near(r.track(0, 20), 0.4, "Live P change affects active tracking");
        r.settings = settings(0.02, 0, 0, 0.05, 0, 0.5);
        near(r.track(0, 20), 0.45, "Live friction feedforward change applies");
        r.settings = settings(0.02, 0, 0, 0.05, 0, 0.3);
        near(r.track(0, 20), 0.3, "Lowered max immediately caps active move");
        r.settings = settings(0.001, 0, 0, 0, 0.25, 0.3);
        near(r.track(0, 20), 0.25, "Raised min immediately affects active move");
        near(r.update(0, 20, false, true, false), 0.25, "Manual movement obeys min speed");
        r.settings = settings(0.001, 0, 0, 0, 0, 0.1);
        near(r.update(0, 20, false, true, false), 0.1, "Manual movement obeys max speed");

        r = new Rig();
        r.settings = settings(0.01, 0.1, 0, 0, 0, 0.5);
        near(r.track(0, 4), 0.06, "I integrates error using elapsed seconds");
        near(r.track(0, 4), 0.08, "I accumulates across active samples");
        near(r.track(4, 4), 0, "Arrival clears accumulated I");
        near(r.track(4, 8), 0.06, "New error starts without old integral");
        r.settings = settings(0.01, 0, 0, 0, 0, 0.5);
        near(r.track(4, 8), 0.04, "Disabling I immediately removes its contribution");
        r.settings = settings(0.01, 0.1, 0, 0, 0, 0.5);
        near(r.track(4, 8), 0.06, "Re-enabled I starts fresh");
        near(r.update(4, 8, false, false, false), 0, "Disabling tracking clears I");
        near(r.track(4, 8), 0.06, "Tracking restart has no stored integral");
        r.controller.rebase(4, Math.round(4 * 8192 * 3.75 / 360), r.now);
        near(r.track(4, 8), 0.06, "Re-zero/rebase clears integral history");

        r = new Rig();
        r.settings = settings(0.1, 0.1, 0, 0, 0, 0.5);
        for (int n = 0; n < 5; n++) near(r.track(0, 20), 0.5, "Saturated output stays capped");
        near(r.track(0, 3), 0.315, "Saturated samples do not wind up the integral");
        near(r.track(0, -3), -0.315, "Error reversal discards old-direction integral");

        for (int direction : new int[]{-1, 1}) {
            r = new Rig();
            r.settings = settings(0.01, 0.1, 0, 0.2, 0.25, 0.5);
            expect(r.track(0, direction * 3) * direction > 0,
                    "Move toward target outside tolerance");
            near(r.track(direction, direction * 3), 0,
                    "Exactly two degrees stops despite minimum power, F, and accumulated I");
            near(r.track(direction * 2, direction * 3), 0, "Stay stopped inside tolerance");
            near(r.track(direction * 5, direction * 3), 0, "Opposite tolerance boundary also stops");
            expect(r.track(direction * 5.01, direction * 3) * direction < 0,
                    "Resume correction just outside tolerance");
        }

        r = new Rig();
        r.settings = settings(0.01, 0, 0, 0, 0, 0.5);
        r.track(0, 10);
        r.settings = settings(0.01, 0, 0.001, 0, 0, 0.5);
        near(r.track(1, 10), 0.08, "Live D damps measured velocity");
        near(r.track(1, 20), 0.185, "Changing target does not cause derivative kick");

        for (TurretAimController.Settings invalid : new TurretAimController.Settings[]{
                settings(0.01, -0.1, 0, 0, 0, 0.5),
                settings(0.01, Double.NaN, 0, 0, 0, 0.5),
                settings(0.01, 0, 0, 0, -0.1, 0.5),
                settings(0.01, 0, 0, 0, 0.6, 0.5),
                settings(0.01, 0, 0, 0, 0, 1.1)}) {
            r = new Rig();
            r.settings = invalid;
            near(r.track(0, 20), 0, "Invalid settings stop movement");
            expect(r.controller.getState().contains("FAULT"), "Invalid settings produce visible fault");
        }
        System.out.println("Passed " + checks + " turret PIDF checks.");
    }
}
