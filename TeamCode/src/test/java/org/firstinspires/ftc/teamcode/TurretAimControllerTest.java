package org.firstinspires.ftc.teamcode;

/** Standalone command-sequence regressions: javac/java, no Android hardware needed. */
public final class TurretAimControllerTest {
    private static final TurretAimController.Settings SETTINGS =
            new TurretAimController.Settings(0.02, 0.001, 0.05, 1, 0.2, 1, 10);
    private static int checks;

    private static void expect(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static final class Rig {
        final TurretAimController controller;
        double now;
        TurretAimController.Settings settings = SETTINGS;

        Rig() {
            this(false, false, false);
        }

        Rig(boolean x, boolean y, boolean b) {
            controller = new TurretAimController(x, y, b);
        }

        double step(double angle, boolean lb, boolean rb, boolean x, boolean y, boolean b) {
            now += 0.05;
            return controller.update(angle, Math.round(angle * 8192 * 3.75 / 360), now,
                    lb, rb, x, y, b, settings);
        }

        double idle(double angle) {
            return step(angle, false, false, false, false, false);
        }

        double track(double angle, double target) {
            now += 0.05;
            return controller.update(angle, Math.round(angle * 8192 * 3.75 / 360), now,
                    false, false, false, false, false, settings, true, target);
        }

        double target(double angle, int target) {
            return step(angle, false, false, target == 90, target == 0, target == -90);
        }

        void reached(double angle) {
            expect(idle(angle) == 0, "Stop at target " + angle);
            now += 0.2;
            expect(idle(angle) == 0, "Remain stopped while settling at " + angle);
            expect(controller.getState().contains("Reached"), "Complete target " + angle);
        }
    }

    public static void main(String[] args) {
        Rig r = new Rig();
        expect(r.step(0, true, false, false, false, false) == 0.2, "LB positive");
        expect(r.step(1, false, true, false, false, false) == -0.2, "RB negative");
        expect(r.step(0, true, true, false, false, false) == 0, "Both bumpers stop");
        expect(r.idle(0) == 0, "Release remains stopped");

        // Use the same controller for repeated X/Y/B moves, alternating with manual
        // movement and overshoot corrections, to catch one-shot completion state.
        r = new Rig();
        for (int cycle = 0; cycle < 3; cycle++) {
            expect(r.target(0, 90) > 0, "X starts positive move, cycle " + cycle);
            expect(r.idle(45) > 0, "X move continues after button release");
            r.reached(90);
            expect(r.step(90, true, false, false, false, false) == 0.2, "LB works after auto completion");
            expect(r.idle(95) == 0, "Manual release does not resume auto");
            expect(r.target(95, 90) < 0, "X repeats and corrects from above 90");
            r.reached(90);
            expect(r.target(90, 0) < 0, "Y goes from positive angle to zero");
            expect(r.idle(45) < 0, "Y continues after release");
            r.controller.rebase(0, 0, r.now); // Physical magnet corrects zero.
            r.reached(0);
            expect(r.target(0, -90) < 0, "B starts negative move");
            expect(r.idle(-45) < 0, "B continues after release");
            r.reached(-90);
            expect(r.step(-90, false, true, false, false, false) == -0.2, "RB works after negative move");
            expect(r.idle(-95) == 0, "RB release stops");
            expect(r.target(-95, -90) > 0, "B repeats and corrects from below -90");
            r.reached(-90);
            expect(r.target(-90, 0) > 0, "Y repeats from negative angle");
            r.reached(0);
        }

        r = new Rig();
        expect(r.target(0, 90) > 0, "Start active X move");
        expect(r.target(10, -90) < 0, "B replaces active X target");
        expect(r.controller.getTargetDegrees() == -90, "Negative target recorded");
        expect(r.target(-10, 0) > 0, "Y replaces active B target");
        expect(r.controller.getTargetDegrees() == 0, "Center target recorded");
        expect(r.step(-5, false, true, false, false, false) < 0, "Manual overrides active Y move");
        expect(r.idle(-6) == 0, "Manual release leaves auto cancelled");

        for (int target : new int[]{90, 0, -90}) {
            r = new Rig(target == 90, target == 0, target == -90);
            expect(r.target(20, target) == 0, "Target held at START does not trigger: " + target);
            r.idle(20);
            expect(r.target(20, target) != 0, "Release/re-press triggers: " + target);
            r.step(20, true, true, target == 90, target == 0, target == -90);
            expect(r.target(20, target) == 0, "Held target does not restart after cancel: " + target);
            r.idle(20);
            expect(r.target(20, target) != 0, "Target can restart after cancel: " + target);
        }

        // A fault stops the current attempt, but never permanently disables input.
        // Keep the same input held to prove that it cannot continually retry itself.
        r = new Rig();
        r.step(0, true, false, false, false, false);
        r.now += 1.1;
        expect(r.step(0, true, false, false, false, false) == 0, "Manual stall stops");
        expect(r.controller.getState().contains("FAULT"), "Stall fault reported");
        expect(r.step(0, true, false, false, false, false) == 0, "Held LB cannot restart stalled attempt");
        r.idle(0);
        expect(r.step(0, true, false, false, false, false) > 0, "Fresh LB recovers from fault");
        expect(!r.controller.getState().contains("FAULT"), "Fresh LB clears old fault");
        r.now += 1.1;
        r.step(0, true, false, false, false, false);
        expect(r.step(0, false, true, false, false, false) < 0, "Fresh RB can recover in opposite direction");

        for (int target : new int[]{90, 0, -90}) {
            r = new Rig();
            double angle = target == 0 ? 30 : 0;
            r.target(angle, target);
            r.now += 1.1;
            expect(r.target(angle, target) == 0, "Stalled target stops: " + target);
            expect(r.controller.getState().contains("FAULT"), "Target stall reported: " + target);
            expect(r.target(angle, target) == 0, "Held target cannot retry fault: " + target);
            r.idle(angle);
            expect(r.target(angle, target) != 0, "Fresh target retries fault: " + target);
        }

        r = new Rig();
        r.step(0, true, false, false, false, false);
        r.now += 0.2;
        expect(r.step(-1, true, false, false, false, false) == 0, "Wrong sign stops attempt");
        expect(r.controller.getState().contains("direction"), "Wrong sign diagnosed");
        expect(r.step(-1, true, true, true, true, true) == 0, "Both bumpers take priority and stop");
        expect(!r.controller.getState().contains("FAULT"), "Both bumpers clear fault");
        expect(r.idle(-1) == 0, "Clearing fault never restarts movement");
        expect(r.target(-1, 90) > 0, "X available after fault cancellation");

        r = new Rig();
        expect(r.step(180, true, false, false, false, false) == 0, "+180 blocks outward");
        expect(r.step(180, false, true, false, false, false) < 0, "+180 permits inward");
        expect(r.step(-180, false, true, false, false, false) == 0, "-180 blocks outward");
        expect(r.step(-180, true, false, false, false, false) > 0, "-180 permits inward");
        expect(r.step(181, true, false, false, false, false) == 0, "Retry cannot bypass positive limit");
        expect(r.step(-181, false, true, false, false, false) == 0, "Retry cannot bypass negative limit");
        expect(r.target(-181, 90) > 0, "Automatic target can recover inward from negative limit");
        expect(r.target(181, -90) < 0, "Automatic target can recover inward from positive limit");

        for (int target : new int[]{90, 0, -90}) {
            for (int direction : new int[]{-1, 1}) {
                r = new Rig();
                r.settings = new TurretAimController.Settings(0.02, 0, 0.05, 1, 0.2, 0.7, 10);
                expect(r.target(target - direction * 10.1, target) == direction * 0.7,
                        "Bang-bang outside range, target " + target + ", direction " + direction);
                expect(r.controller.getState().contains("Bang-bang"), "Bang-bang phase reported");
                r = new Rig();
                r.settings = new TurretAimController.Settings(0.02, 0, 0.05, 1, 0.2, 0.7, 10);
                expect(r.target(target - direction * 10, target) == direction * 0.25,
                        "PDF at boundary, target " + target + ", direction " + direction);
                expect(r.controller.getState().contains("PDF"), "PDF phase reported");
            }
        }

        r = new Rig();
        r.settings = new TurretAimController.Settings(0.02, 0, 0.05, 1, 0.2, 0.7, 5);
        expect(r.target(84, 90) == 0.7, "Editable range keeps bang-bang at 6 degree error");
        r.settings = new TurretAimController.Settings(0.02, 0, 0.05, 1, 0.2, 0.7, 10);
        expect(Math.abs(r.idle(84) - 0.17) < 1e-9, "Live range change switches active move to PDF");

        r = new Rig();
        r.target(-20, 90);
        r.controller.rebase(0, 0, r.now);
        expect(r.idle(0) > 0, "Magnet rebase preserves active target");
        expect(r.controller.getVelocity() == 0, "Magnet rebase prevents derivative spike");
        expect(!r.controller.getState().contains("FAULT"), "Magnet rebase avoids false direction fault");

        r = new Rig();
        r.settings = new TurretAimController.Settings(2, 0, 0.1, 0.3, 0.8, 1, 10);
        expect(r.target(0, 90) == 0.3, "Bang-bang capped");
        expect(r.step(0, false, true, false, false, false) == -0.3, "Manual capped");
        for (TurretAimController.Settings invalid : new TurretAimController.Settings[]{
                new TurretAimController.Settings(Double.NaN, 0, 0, 1, 0.2, 1, 10),
                new TurretAimController.Settings(0.02, 0, 0, 1, 0.2, 1, -1),
                new TurretAimController.Settings(0.02, 0, 0, 1, 0.2, Double.NaN, 10)}) {
            r = new Rig();
            r.settings = invalid;
            expect(r.target(0, 90) == 0, "Invalid settings stop automatic movement");
            expect(r.step(0, true, false, false, false, false) == 0, "Fresh manual cannot bypass invalid settings");
        }
        expect(Math.abs(TurretAimController.fieldTargetDegrees(0, 0, 0, 1, 1, 0, 0) + 45) < 1e-9,
                "Goal to left requires negative clockwise turret angle");
        expect(Math.abs(TurretAimController.fieldTargetDegrees(0, 0, Math.PI / 2, 1, 0, 0, 0) - 90) < 1e-9,
                "Left chassis turn requires right turret turn");
        expect(Math.abs(TurretAimController.fieldTargetDegrees(0, 1, 0, 1, 1, 0, 0)) < 1e-9,
                "Translation updates goal bearing");
        expect(Math.abs(TurretAimController.fieldTargetDegrees(0, 0, Math.PI / 2, 1, 1, 1, 0) - 90) < 1e-9,
                "Pivot offset rotates with chassis");
        expect(Double.isNaN(TurretAimController.fieldTargetDegrees(1, 1, 0, 1, 1, 0, 0)),
                "Coincident goal cannot define a bearing");
        expect(Double.isNaN(TurretAimController.fieldTargetDegrees(0, 0, 0, Double.POSITIVE_INFINITY, 1, 0, 0)),
                "Nonfinite field coordinates rejected");
        r = new Rig();
        expect(r.track(0, 45) > 0, "Tracking starts toward goal");
        expect(r.track(45, 45) == 0, "Tracking stops power when aligned");
        r.now += 0.3;
        expect(r.track(45, 45) == 0, "Tracking stays aligned past preset settle time");
        expect(r.track(45, 60) > 0, "Tracking follows changing bearing after settling");
        expect(r.idle(46) == 0, "Toggle off cancels tracking motion");
        r = new Rig();
        r.track(0, 45);
        r.now += 1.1;
        expect(r.track(0, 50) == 0, "Moving targets retain stall history");
        expect(r.track(0, 60) == 0, "Tracking cannot retry a latched stall");
        r.idle(0);
        expect(r.track(0, 60) > 0, "Toggle off/on retries fault");
        expect(r.track(1, Double.NaN) == 0, "Invalid pose or zero stops tracking");
        expect(r.track(1, 60) == 0, "Recovery requires explicit retry");
        r = new Rig();
        r.track(179, 179);
        expect(r.track(179, -179) == 0, "Rear boundary prevents full-turn reversal");
        expect(r.controller.getState().contains("rear travel"), "Rear boundary explained");
        r = new Rig();
        r.track(0, 45);
        expect(r.step(1, false, true, false, false, false) < 0, "Manual overrides tracking");
        expect(r.idle(0) == 0, "Manual release leaves tracking cancelled");
        System.out.println("Passed " + checks + " turret controller regression checks.");
    }
}
