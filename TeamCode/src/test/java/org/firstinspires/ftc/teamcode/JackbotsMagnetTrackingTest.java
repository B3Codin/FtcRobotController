package org.firstinspires.ftc.teamcode;

/** Startup homing and subsequent encoder-only tracking in the actual TeleOp controller. */
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
        r.detected = true;
        r.turret.initSample(12345, false);
        expect(r.step(0) == 0, "No motion during INIT even when sensor is active");
        expect(r.turret.angleDegrees() == 0, "INIT does not apply magnet angle");
        r.detected = false;
        r.start();
        expect(r.step(0) == 0.1, "START seeks in the positive direction at exactly 0.1");
        expect(r.step(1, -45, false, false) == 0.1,
                "Homing precedes normal aiming without manual input");
        r.detected = true;
        expect(r.step(2) == -0.1, "Sensor detection reverses toward zero at 0.1");
        expect(r.turret.angleDegrees() == 10.2494, "Preserve the configured sensor angle");
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
        r.detected = false;
        r.step(rawZeroDegrees + 1);
        r.detected = true;
        r.step(rawZeroDegrees + 2);
        expect(Math.abs(r.turret.angleDegrees() - 2) < 0.02,
                "Later negative-side sensor entries cannot change the reference");
        r.turret.onStart(false, false);
        expect(r.step(rawZeroDegrees + 3) > 0 && !r.turret.state().startsWith("Homing:"),
                "Repeated start notifications cannot restart homing");
        expect(r.step(rawZeroDegrees + 4, 45, true, false) == -0.2,
                "Normal manual controls resume after homing");
        expect(r.step(rawZeroDegrees + 3) == 0, "Manual override still cancels auto aim");

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
