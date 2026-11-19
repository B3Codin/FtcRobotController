package org.firstinspires.ftc.teamcode;

/** Standalone regression checks: compile/run with javac/java, no Android hardware needed. */
public final class TurretAimControllerTest {
    private static final TurretAimController.Settings SETTINGS =
            new TurretAimController.Settings(0.02, 0.001, 0.05, 1.0, 0.2);
    private static int checks;

    private static void expect(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static double step(TurretAimController c, double angle, long ticks, double now,
                               boolean lb, boolean rb, boolean x, boolean b) {
        return c.update(angle, ticks, now, lb, rb, x, b, SETTINGS);
    }

    public static void main(String[] args) {
        TurretAimController c = new TurretAimController(false);
        expect(step(c, 0, 0, 0, true, false, false, false) == 0.2, "LB positive");
        expect(step(c, 0, 0, 0.01, false, true, false, false) == -0.2, "RB negative");
        expect(step(c, 0, 0, 0.02, true, true, false, false) == 0, "Both bumpers stop");
        expect(step(c, 0, 0, 0.03, false, false, false, false) == 0, "Release stops");

        c = new TurretAimController(false);
        expect(step(c, 180, 1000, 0, true, false, false, false) == 0, "+180 blocks outward");
        expect(step(c, 180, 1000, 0.01, false, true, false, false) < 0, "+180 permits inward");
        expect(step(c, -180, -1000, 0.02, false, true, false, false) == 0, "-180 blocks outward");
        expect(step(c, -180, -1000, 0.03, true, false, false, false) > 0, "-180 permits inward");
        expect(step(c, 181, 1100, 0.04, true, false, false, false) == 0, "Overtravel still blocked");
        expect(step(c, -181, -1100, 0.05, false, true, false, false) == 0, "Negative overtravel blocked");

        c = new TurretAimController(false);
        expect(step(c, 0, 0, 0, false, false, true, false) > 0, "X starts auto");
        expect(step(c, 20, 100, 0.1, false, true, false, false) < 0, "Manual overrides auto");
        expect(step(c, 20, 100, 0.2, false, false, false, false) == 0, "Manual release does not resume auto");

        c = new TurretAimController(false);
        expect(step(c, 100, 1000, 0, false, false, true, false) < 0, "X corrects from above 90");
        step(c, 90, 900, 0.1, false, false, false, false);
        step(c, 90, 900, 0.3, false, false, false, false);
        expect(c.getState().contains("Reached"), "90 settles and completes");
        expect(step(c, 90, 900, 0.4, false, true, false, false) < 0, "Can move away after completion");
        step(c, 0, 0, 0.5, false, false, false, false);
        expect(step(c, 0, 0, 0.6, false, false, true, false) > 0, "Second X repeats 90 move");

        c = new TurretAimController(false);
        step(c, -10, -100, 0, false, false, true, false);
        c.rebase(0, 0, 0.2); // Magnet changed angle/count reference during an active move.
        expect(step(c, 0, 0, 0.2, false, false, false, false) > 0, "Magnet rebase preserves auto");
        expect(c.getVelocity() == 0, "Magnet rebase removes D spike");
        expect(!c.getState().contains("FAULT"), "Magnet rebase is not a direction fault");

        c = new TurretAimController(false);
        step(c, 0, 0, 0, true, false, false, false);
        expect(step(c, 0, 0, 1.1, true, false, false, false) == 0, "Stalled manual stops");
        expect(c.getState().contains("FAULT"), "Stall fault shown");
        expect(step(c, 0, 0, 1.2, false, false, true, false) == 0, "X cannot override fault");
        expect(step(c, 0, 0, 1.3, true, false, false, true) == 0, "B clears and stops even with LB");
        expect(step(c, 0, 0, 1.4, false, false, true, false) > 0, "X works after fault cleared");

        c = new TurretAimController(false);
        step(c, 0, 0, 0, true, false, false, false);
        expect(step(c, -1, -100, 0.2, true, false, false, false) == 0, "Wrong encoder sign stops");
        expect(c.getState().contains("direction"), "Wrong sign diagnosed");

        c = new TurretAimController(true);
        expect(step(c, 0, 0, 0, false, false, true, false) == 0, "X held at START does not trigger");
        step(c, 0, 0, 0.1, false, false, false, false);
        expect(step(c, 0, 0, 0.2, false, false, true, false) > 0, "Released/repressed X triggers");
        expect(step(c, 10, 100, 0.3, false, false, true, true) == 0, "B cancels auto");
        expect(step(c, 10, 100, 0.4, false, false, true, false) == 0, "Holding X after B does not restart");

        c = new TurretAimController(false);
        TurretAimController.Settings capped = new TurretAimController.Settings(2, 0, 0.1, 0.3, 0.8);
        expect(c.update(0, 0, 0, false, false, true, false, capped) == 0.3, "Auto power capped");
        expect(c.update(0, 0, 0.1, false, true, false, false, capped) == -0.3, "Manual power capped");
        TurretAimController.Settings invalid = new TurretAimController.Settings(Double.NaN, 0, 0, 1, 0.2);
        expect(c.update(0, 0, 0.2, true, false, false, false, invalid) == 0, "Invalid gains stop turret");

        System.out.println("Passed " + checks + " turret controller regression checks.");
    }
}
