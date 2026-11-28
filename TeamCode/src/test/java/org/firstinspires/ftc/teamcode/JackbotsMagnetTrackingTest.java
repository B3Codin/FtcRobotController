package org.firstinspires.ftc.teamcode;

/** Startup homing and ongoing magnet corrections in the actual TeleOp controller. */
public final class JackbotsMagnetTrackingTest {
    private static int checks;

    private static void expect(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static final class Rig {
        final JackbotsTeleOp.TurretTracking turret = new JackbotsTeleOp.TurretTracking(12345, false);
        final TurretAimController.Settings settings =
                new TurretAimController.Settings(0.002, 0.0, 0.0, 0.20, 0.20, 1.0, 0.2, 1.0, 360.0);
        double now;
        double magnetAngle = 10.2494;
        boolean detected;

        void start() {
            turret.onStart(false, false);
        }

        double step(double encoderDegrees, double target, boolean lb, boolean rb) {
            now += 0.05;
            int ticks = 12345 + (int) Math.round(encoderDegrees * TurretMagnetZero.TICKS_PER_DEGREE);
            return turret.update(ticks, now, true, target, false, false, lb, rb,
                    detected, magnetAngle, settings);
        }

        double step(double encoderDegrees) {
            return step(encoderDegrees, 45, false, false);
        }

        void homeAtStart() {
            detected = true;
            magnetAngle = 0;
            start();
            step(0);
            now += 0.2;
            step(0);
        }
    }

    public static void main(String[] args) {
        Rig r = new Rig();
        expect(!r.turret.needsMagnetSample(), "INIT does not read the magnet");
        r.detected = true;
        r.turret.initSample(12345, false);
        expect(r.step(0) == 0, "No motion during INIT even when sensor is active");
        expect(r.turret.angleDegrees() == 0, "INIT does not apply magnet angle");
        r.detected = false;
        r.start();
        expect(r.turret.needsMagnetSample(), "Startup seek reads the magnet");
        expect(r.step(0) == 0.1, "START seeks in the positive direction at exactly 0.1");
        expect(r.step(1, -45, false, false) == 0.1,
                "Homing precedes normal aiming without manual input");
        r.detected = true;
        expect(r.step(2) == -0.1, "Sensor detection reverses toward zero at 0.1");
        expect(r.turret.angleDegrees() == 10.2494, "Preserve the configured sensor angle");
        expect(r.turret.needsMagnetSample(), "Return samples sensor edges without changing its reference");
        expect(r.turret.controller.getTargetDegrees() == 0, "Return targets zero rather than the field goal");
        r.magnetAngle = 80;
        expect(r.step(-3) == -0.1, "Return continues while sensor stays active");
        expect(Math.abs(r.turret.angleDegrees() - 5.2494) < 0.02,
                "Sensor and setting changes cannot rebase the return");
        double rawZeroDegrees = 2 - 10.2494;
        expect(r.step(rawZeroDegrees) == 0, "Arrival within one degree stops turret power");
        r.now += 0.2;
        expect(r.step(rawZeroDegrees) == 0, "Zero must settle before homing completes");
        expect(r.step(rawZeroDegrees) > 0, "Normal field aiming resumes after homing");
        expect(r.turret.needsMagnetSample(), "Normal aiming continues reading the magnet");
        r.magnetAngle = 10.2494;
        r.detected = false;
        r.step(rawZeroDegrees + 1);
        r.detected = true;
        expect(r.step(rawZeroDegrees + 2) > 0, "Auto aiming continues across a magnet correction");
        expect(r.turret.angleDegrees() == r.magnetAngle,
                "Later negative-side sensor entries correct the angle to the configured reference");
        expect(r.turret.controller.getTargetDegrees() == 45, "Correction preserves the field target");
        r.turret.onStart(false, false);
        expect(r.step(rawZeroDegrees + 3) > 0 && !r.turret.state().startsWith("Homing:"),
                "Repeated start notifications cannot restart homing");
        expect(Math.abs(r.turret.angleDegrees() - 11.2494) < 0.02,
                "A held magnet does not pin the angle at its reference");
        expect(r.step(rawZeroDegrees + 4, 45, true, false) == -0.2,
                "Normal manual controls resume after homing");
        expect(r.step(rawZeroDegrees + 3) == 0, "Manual override still cancels auto aim");
        expect(r.turret.needsMagnetSample(), "Manual override keeps magnet reads enabled");

        int postHomeTicks = 12345 + (int) Math.round((rawZeroDegrees + 3)
                * TurretMagnetZero.TICKS_PER_DEGREE);
        double angleBeforeToggle = r.turret.angleDegrees();
        expect(r.turret.update(postHomeTicks, r.now + 0.05, true, 45,
                false, true, false, false, true, Double.NaN, r.settings) > 0,
                "Auto resumes after manual override despite invalid later magnet data");
        expect(r.turret.angleDegrees() == angleBeforeToggle && r.turret.needsMagnetSample(),
                "Re-enabling auto ignores invalid magnet data and keeps sensor reads enabled");
        expect(r.turret.update(postHomeTicks, r.now + 1.2, true, 45,
                false, false, false, false, true, 100, r.settings) == 0,
                "A later magnet trigger cannot prevent the normal encoder stall fault");
        expect(r.turret.state().contains("No turret movement") && r.turret.needsMagnetSample(),
                "A held magnet does not hide a tracking stall");
        r.turret.update(postHomeTicks, r.now + 1.25, true, 45,
                false, true, false, false, false, 100, r.settings);
        r.turret.update(postHomeTicks, r.now + 1.30, true, 45,
                false, false, false, false, false, 100, r.settings);
        expect(r.turret.update(postHomeTicks, r.now + 1.35, true, 45,
                false, true, false, false, true, 100, r.settings) > 0,
                "Retrying a tracking fault resumes aiming");
        expect(r.turret.angleDegrees() == angleBeforeToggle && r.turret.needsMagnetSample(),
                "Fault recovery does not correct a sensor entry approached from the positive side");

        r = new Rig();
        r.homeAtStart();
        r.magnetAngle = 10;
        r.step(1);
        expect(Math.abs(r.turret.angleDegrees() - 1) < 0.02,
                "Sensor held through startup cannot cause a false entry after homing");
        r.detected = false;
        r.step(2, 45, false, true);
        r.detected = true;
        expect(r.step(3, 45, false, true) == 0.2 && r.turret.angleDegrees() == 10,
                "Manual motion also applies magnet corrections without changing power");
        r.detected = false;
        r.step(5, 45, true, false);
        r.detected = true;
        r.step(4, 45, true, false);
        expect(Math.abs(r.turret.angleDegrees() - 11) < 0.02,
                "Positive-side entry preserves the angle reference");

        r.detected = false;
        r.step(3);
        r.detected = true;
        r.magnetAngle = 181;
        r.step(4);
        expect(Math.abs(r.turret.angleDegrees() - 11) < 0.02,
                "An out-of-range magnet setting cannot corrupt the reference");
        r.detected = false;
        r.step(3);
        r.detected = true;
        r.magnetAngle = Double.NaN;
        r.step(4);
        expect(Math.abs(r.turret.angleDegrees() - 11) < 0.02,
                "A nonfinite magnet setting cannot corrupt the reference");

        r = new Rig();
        r.homeAtStart();
        r.detected = false;
        r.step(1);
        r.now += 1.1;
        r.step(1);
        r.detected = true;
        r.magnetAngle = 10;
        expect(r.step(2) == 0 && r.turret.angleDegrees() == 10,
                "A valid magnet entry can correct the angle while faulted without restarting motion");
        expect(r.turret.state().contains("No turret movement"),
                "Magnet correction preserves the latched controller fault");

        r = new Rig();
        r.homeAtStart();
        r.detected = false;
        r.step(1);
        int manualZeroTicks = 12345 + (int) Math.round(2 * TurretMagnetZero.TICKS_PER_DEGREE);
        expect(r.turret.update(manualZeroTicks, r.now + 0.05, true, 45,
                true, false, false, false, true, 10, r.settings) == 0,
                "Manual zero stops motion when a magnet entry arrives simultaneously");
        expect(r.turret.angleDegrees() == 0, "Manual zero takes priority over magnet correction");
        r.turret.update(manualZeroTicks, r.now + 0.10, true, 45,
                false, false, false, false, true, 10, r.settings);
        expect(r.turret.angleDegrees() == 0, "Held magnet cannot overwrite the driver's zero on release");

        r = new Rig();
        r.detected = true;
        r.start();
        expect(r.step(0) == -0.1, "Starting on the sensor returns immediately without seeking farther");

        r = new Rig();
        r.homeAtStart();
        expect(r.turret.angleDegrees() == 0, "A zero-degree sensor setting needs no return movement");
        expect(r.step(0) > 0, "Zero-degree homing hands control back to aiming");

        r = new Rig();
        r.detected = true;
        r.magnetAngle = -10;
        r.start();
        expect(r.step(0) == 0.1, "A negative sensor angle returns in the positive direction");

        r = new Rig();
        r.start();
        r.step(0);
        r.now += 1.1;
        expect(r.step(0) == 0 && r.turret.state().contains("No turret movement"),
                "A stalled seek stops and latches a fault");
        r.detected = true;
        expect(r.step(1) == 0, "A later sensor detection cannot restart failed homing");

        r = new Rig();
        r.start();
        r.step(0);
        r.now += 0.2;
        expect(r.step(-1) == 0 && r.turret.state().contains("direction"),
                "Wrong encoder direction stops the seek");

        r = new Rig();
        r.start();
        r.step(0);
        r.now += 15;
        expect(r.step(1) == 0 && r.turret.state().contains("Timed out"), "Missing sensor times out");

        r = new Rig();
        r.start();
        r.step(0);
        expect(r.step(360) == 0 && r.turret.state().contains("one turret revolution"),
                "A missed sensor cannot cause repeated revolutions");

        r = new Rig();
        r.start();
        expect(r.step(0, 45, true, true) == 0 && !r.turret.state().startsWith("Homing:"),
                "Both bumpers abort homing");
        expect(r.step(1) == 0, "Releasing bumpers cannot restart cancelled homing");

        r = new Rig();
        r.start();
        r.magnetAngle = Double.NaN;
        expect(r.step(0) == 0 && r.turret.state().contains("Invalid magnet angle"),
                "Invalid reference cannot start motion");

        r = new Rig();
        r.detected = true;
        r.start();
        r.step(0);
        r.now += 1.1;
        expect(r.step(0) == 0 && r.turret.state().contains("No turret movement"),
                "A stalled return also stops and latches a fault");

        expect(r.step(0, 45, true, false) == -0.2,
                "LB recovers manual control after failed return");
        expect(r.step(-1) == 0, "Manual release after homing failure stays stopped");

        r = new Rig();
        r.start();
        r.step(0);
        expect(r.step(1, 45, true, false) == -0.2,
                "LB immediately interrupts homing");
        expect(r.step(0) == 0, "Homing does not resume after manual release");
        expect(r.step(0, 45, false, true) == 0.2,
                "RB works without a magnet calibration");

        r = new Rig();
        r.start();
        r.step(0);
        r.now += 1.1;
        r.step(0);
        expect(r.step(0, 45, false, true) == 0.2,
                "RB clears a stalled seek even though seek used the same direction");

        r = new Rig();
        r.start();
        r.step(0);
        r.now += 15;
        r.step(1);
        expect(r.turret.update(12345, r.now + 0.05, false, Double.NaN,
                false, true, false, false, false, r.magnetAngle, r.settings) == 0.1,
                "D-pad Up retries failed homing without needing odometry READY");

        r = new Rig();
        r.start();
        r.step(0);
        expect(r.turret.update(12345, r.now + 0.05, false, Double.NaN,
                false, true, false, false, false, r.magnetAngle, r.settings) == 0,
                "D-pad Up toggles off during homing");
        expect(r.turret.update(12345, r.now + 0.10, false, Double.NaN,
                false, true, false, false, false, r.magnetAngle, r.settings) == 0,
                "Holding D-pad Up cannot restart homing");
        r.turret.update(12345, r.now + 0.15, false, Double.NaN,
                false, false, false, false, false, r.magnetAngle, r.settings);
        expect(r.turret.update(12345, r.now + 0.20, false, Double.NaN,
                false, true, false, false, false, r.magnetAngle, r.settings) == 0.1,
                "A fresh D-pad Up press restarts uncalibrated homing");

        r = new Rig();
        r.start();
        r.step(0);
        expect(r.turret.update(12345, r.now + 0.05, false, Double.NaN,
                true, false, false, false, false, r.magnetAngle, r.settings) == 0,
                "D-pad Down overrides homing and sets a straight reference");
        expect(r.turret.angleDegrees() == 0, "Manual zero is applied during homing");
        expect(r.turret.update(12345, r.now + 0.10, false, Double.NaN,
                false, false, true, false, false, r.magnetAngle, r.settings) == -0.2,
                "Manual control works while odometry is unavailable");
        System.out.println("Passed " + checks + " Jackbots homing checks.");
    }
}
