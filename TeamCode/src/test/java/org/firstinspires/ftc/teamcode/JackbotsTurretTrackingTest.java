package org.firstinspires.ftc.teamcode;

/** Standalone tests of JackbotsTeleOp's actual encoder/auto/manual coordination. */
public final class JackbotsTurretTrackingTest {
    // Keep controller direction checks independent of the field-coordinate example.
    private static final double START_TARGET = -55.92;
    private static int checks;

    private static void expect(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static final class Rig {
        // A nonzero Hub count proves calibration does not assume a reset motor channel.
        final JackbotsTeleOp.TurretTracking turret = new JackbotsTeleOp.TurretTracking(12345, false);
        TurretAimController.Settings settings =
                new TurretAimController.Settings(0.002, 0.0, 0.0, 0.20, 0.20, 1.0, 0.2, 1.0, 360.0);
        double now;

        void calibrate() {
            turret.initSample(12345, true);
            turret.initSample(12345, false);
            turret.onStart(false, false);
            // Complete startup homing with the sensor already at zero.
            turret.update(12345, now, true, 0, false, false, false, false, true, 0, settings);
            now += 0.2;
            turret.update(12345, now, true, 0, false, false, false, false, true, 0, settings);
        }

        double step(double angle, double target, boolean ready, boolean zero,
                    boolean toggle, boolean lb, boolean rb) {
            now += 0.05;
            int rawTicks = 12345 + (int) Math.round(angle * 8192.0 * 3.75 / 360.0);
            return turret.update(rawTicks, now, ready, target, zero, toggle, lb, rb,
                    false, 0.0, settings);
        }

        double track(double angle, double target) {
            return step(angle, target, true, false, false, false, false);
        }
    }

    public static void main(String[] args) {
        JackbotsTeleOp.TurretTarget example = new JackbotsTeleOp.TurretTarget(
                357, -1687, Math.toRadians(42.9));
        expect(!example.useGoalTwo, "Example selects goal one");
        expect(Math.abs(example.diffX - 1040.6) < 1e-9, "Example DiffX is goal minus robot");
        expect(Math.abs(example.diffY - 216) < 1e-9, "Example DiffY is goal minus robot");
        expect(Math.abs(example.turretGoalX - 1075.8) < 0.05, "Example TurretGoalX rounds to 1075.8");
        expect(Math.abs(example.turretGoalY + 248.7) < 0.05, "Example TurretGoalY rounds to -248.7");
        expect(Math.abs(example.idealTheta - 29.88) < 0.005, "Example IdealTheta rounds to 29.88");
        expect(!new JackbotsTeleOp.TurretTarget(1829, 0, 0).useGoalTwo,
                "Exact selection boundary retains goal one");
        JackbotsTeleOp.TurretTarget secondGoal = new JackbotsTeleOp.TurretTarget(2500, -1000, 0);
        expect(secondGoal.useGoalTwo && secondGoal.goalX == 2260, "Above boundary selects goal two");
        expect(secondGoal.diffX == -240 && secondGoal.diffY == -471,
                "Differences remain signed when robot passes the goal");
        expect(Double.isNaN(new JackbotsTeleOp.TurretTarget(1445.6, -1471, 0).idealTheta),
                "Coincident turret/goal does not command an arbitrary angle");
        expect(Double.isNaN(new JackbotsTeleOp.TurretTarget(Double.NaN, 0, 0).idealTheta),
                "Invalid position produces an invalid target");
        JackbotsTeleOp.TurretTarget wrapped = new JackbotsTeleOp.TurretTarget(357, -1687, Math.PI);
        expect(wrapped.idealTheta > 90 && wrapped.idealTheta < 180,
                "Target below -180 wraps into the positive half of the travel range");

        Rig r = new Rig();
        expect(r.track(0, START_TARGET) == 0, "No automatic motion before straight zero");
        r.calibrate();
        expect(r.step(0, START_TARGET, false, false, false, false, false) == 0,
                "Calibrated turret waits for odometry READY");
        expect(Math.abs(r.track(0, START_TARGET) - (0.002 * START_TARGET - 0.2)) < 1e-9,
                "Auto uses PIDF even outside the former 30-degree bang-bang range");
        expect(r.track(-20, START_TARGET) < 0, "Tracking continues with new encoder samples");
        expect(r.track(START_TARGET, START_TARGET) == 0, "Encoder arrival stops both-servo command");
        expect(Math.abs(r.turret.angleDegrees() - START_TARGET) < 0.006,
                "8192-count encoder and 90/24 gearing convert to turret degrees");
        expect(r.track(START_TARGET, -20) > 0, "Changing IdealTheta resumes tracking after alignment");

        r = new Rig();
        r.calibrate();
        r.track(0, START_TARGET);
        expect(r.step(-1, START_TARGET, true, false, false, true, false) == -0.2,
                "LB overrides auto with original negative manual power");
        expect(r.track(-2, START_TARGET) == 0, "Manual release keeps auto disabled");
        expect(r.step(-2, START_TARGET, true, false, true, false, false) < 0,
                "D-pad Up resumes auto after manual override");
        expect(r.step(-3, START_TARGET, true, false, true, false, false) < 0,
                "Holding toggle does not toggle repeatedly");
        expect(r.step(-4, START_TARGET, true, false, false, false, true) == 0.2,
                "RB preserves positive manual direction");
        expect(r.step(-3, START_TARGET, true, false, false, true, true) == 0,
                "Both bumpers stop");
        expect(r.track(-3, START_TARGET) == 0, "Both-bumper release stays stopped");

        r = new Rig();
        r.calibrate();
        r.track(0, START_TARGET);
        expect(r.step(-1, START_TARGET, false, false, false, false, false) == 0,
                "Loss of odometry READY stops an active move");
        expect(r.track(-1, START_TARGET) == 0, "Restored odometry cannot silently retry fault");
        expect(r.turret.state().contains("FAULT"), "Odometry fault is visible");
        expect(r.step(-1, START_TARGET, true, false, true, false, false) == 0, "Toggle off");
        r.track(-1, START_TARGET);
        expect(r.step(-1, START_TARGET, true, false, true, false, false) < 0,
                "Toggle off/on explicitly retries tracking");

        r = new Rig();
        r.calibrate();
        r.track(0, START_TARGET);
        r.now += 1.1;
        expect(r.track(0, START_TARGET) == 0, "Frozen encoder latches stall");
        expect(r.track(0, -50) == 0, "Changing IdealTheta cannot clear stall");
        expect(r.turret.state().contains("No turret movement"), "Stall is diagnosed");

        r = new Rig();
        r.calibrate();
        r.track(0, START_TARGET);
        r.now += 0.2;
        expect(r.track(1, START_TARGET) == 0, "Wrong encoder direction stops auto");
        expect(r.turret.state().contains("direction"), "Direction fault is visible");

        r = new Rig();
        r.calibrate();
        expect(r.track(179, 179) == 0, "Aligned near rear limit");
        expect(r.track(179, -179) == 0, "Target wrapping cannot command a full-turn reversal");
        expect(r.turret.state().contains("rear travel"), "Rear boundary fault retained");
        expect(r.step(180, 0, true, false, false, false, true) == 0, "RB blocked at +180");
        expect(r.step(180, 0, true, false, false, true, false) < 0, "LB recovers inward");
        expect(r.step(-180, 0, true, false, false, true, false) == 0, "LB blocked at -180");
        expect(r.step(-180, 0, true, false, false, false, true) > 0, "RB recovers inward");

        r = new Rig();
        r.calibrate();
        r.track(0, START_TARGET);
        expect(r.step(-5, START_TARGET, true, true, false, true, false) == 0,
                "Zero calibration stops auto and manual power");
        expect(r.turret.angleDegrees() == 0, "Physical mark resets encoder reference");
        expect(r.step(-5, START_TARGET, true, true, false, false, false) == 0,
                "Holding zero keeps turret stopped");

        r = new Rig();
        r.calibrate();
        r.turret.onStart(false, true);
        expect(r.step(0, START_TARGET, true, false, true, false, false) < 0,
                "Toggle held at START does not disable default automatic aiming");
        expect(r.track(-1, Double.NaN) == 0, "Undefined IdealTheta stops movement");
        System.out.println("Passed " + checks + " Jackbots turret tracking checks.");
    }
}
