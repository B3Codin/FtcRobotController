package org.firstinspires.ftc.teamcode;

/** Checks the magnet correction in TeleOp's actual auto/manual control path. */
public final class JackbotsMagnetTrackingTest {
    private static int checks;

    private static void expect(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static final class Rig {
        // A nonzero Hub count proves calibration does not assume a reset motor channel.
        final JackbotsTeleOp.TurretTracking turret = new JackbotsTeleOp.TurretTracking(12345, false, false);
        TurretAimController.Settings settings =
                new TurretAimController.Settings(0.002, 0.0, 0.0, 0.20, 0.20, 1.0, 0.2, 1.0, 360.0);
        double now;
        boolean magnetDetected;
        double magnetAngleDegrees;

        void calibrate() {
            turret.initSample(12345, true, magnetDetected, magnetAngleDegrees);
            turret.initSample(12345, false, magnetDetected, magnetAngleDegrees);
            turret.onStart(false, false);
        }

        double step(double angle, double target, boolean ready, boolean zero,
                    boolean toggle, boolean lb, boolean rb) {
            now += 0.05;
            int rawTicks = 12345 + (int) Math.round(angle * 8192.0 * 3.75 / 360.0);
            return turret.update(rawTicks, now, ready, target, zero, toggle, lb, rb,
                    magnetDetected, magnetAngleDegrees, settings);
        }

        double track(double angle, double target) {
            return step(angle, target, true, false, false, false, false);
        }
    }

    public static void main(String[] args) {
        Rig r;
        r = new Rig();
        r.calibrate();
        r.track(-10, 45);
        r.magnetAngleDegrees = -2.75;
        r.magnetDetected = true;
        expect(r.track(-5, 45) > 0, "Magnet correction preserves auto aim in the same loop");
        expect(r.turret.angleDegrees() == -2.75, "Magnet assigns the configured angle exactly");
        expect(r.turret.controller.getVelocity() == 0, "Correction does not cause a derivative spike");
        expect(r.turret.controller.getTargetDegrees() == 45, "Magnet does not replace the aiming target");
        expect(r.track(-4, 45) > 0 && r.turret.angleDegrees() > -2.75,
                "Tracking continues while the magnet stays on");
        double correctedAngle = r.turret.angleDegrees();
        r.magnetAngleDegrees = 12.5;
        r.track(-4, 45);
        expect(r.turret.angleDegrees() == correctedAngle, "Live setting edits wait for the next entry");
        r.magnetDetected = false;
        r.track(10, 45);
        r.magnetDetected = true;
        r.track(5, -45);
        expect(Math.abs(r.turret.angleDegrees() - 7.25) < 0.02,
                "Positive-side entry does not apply the new angle");
        r.magnetDetected = false;
        r.track(-10, -45);
        r.magnetDetected = true;
        expect(r.track(-5, 45) > 0 && r.turret.angleDegrees() == 12.5,
                "Next negative-side entry uses the new configured angle");
        expect(r.step(-5, 45, true, true, false, false, false) == 0
                        && r.turret.angleDegrees() == 0,
                "D-pad Down still sets physical straight zero with a nonzero magnet setting");

        r = new Rig();
        expect(r.track(-10, 45) == 0, "Uncalibrated turret waits before first magnet entry");
        r.magnetDetected = true;
        expect(r.track(-5, 45) > 0 && r.turret.angleDegrees() == 0,
                "First valid magnet entry establishes zero and permits auto aim");

        r = new Rig();
        r.calibrate();
        r.step(-10, 45, true, false, false, false, true);
        r.magnetDetected = true;
        expect(r.step(-5, 45, true, false, false, false, true) == 0.2,
                "Magnet correction keeps a held manual bumper moving");
        expect(r.track(-4, 45) == 0, "Magnet correction cannot re-enable manually cancelled auto aim");

        r = new Rig();
        r.calibrate();
        r.track(-10, 45);
        r.now += 1.1;
        expect(r.track(-10, 45) == 0 && r.turret.state().contains("FAULT"), "Create a stall fault");
        r.magnetDetected = true;
        expect(r.track(-5, 45) == 0 && r.turret.state().contains("FAULT"),
                "Magnet correction preserves a latched fault");

        JackbotsTeleOp.TurretTracking initTurret = new JackbotsTeleOp.TurretTracking(100, false, true);
        initTurret.initSample(110, false, true, 8.5);
        expect(Math.abs(initTurret.angleDegrees() - 10 / TurretMagnetZero.TICKS_PER_DEGREE) < 1e-9,
                "Starting on the magnet does not apply a correction");
        initTurret.initSample(90, false, false, 8.5);
        initTurret.initSample(100, false, true, 8.5);
        expect(initTurret.angleDegrees() == 8.5, "INIT samples can establish the magnet reference");
        initTurret.onStart(false, false);
        expect(initTurret.update(110, 0.05, true, 45, false, false, false, false,
                        true, 8.5, r.settings) > 0,
                "A calibration made during INIT permits aiming after START");
        System.out.println("Passed " + checks + " Jackbots magnet tracking checks.");
    }
}
