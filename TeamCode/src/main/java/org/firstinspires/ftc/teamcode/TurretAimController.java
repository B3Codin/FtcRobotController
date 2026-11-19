package org.firstinspires.ftc.teamcode;

/** Hardware-independent turret control. Positive power must increase signed encoder ticks. */
final class TurretAimController {
    static final double LIMIT_DEGREES = 180.0;
    private static final double ANGLE_TOLERANCE_DEGREES = 0.5;
    private static final double SETTLE_SECONDS = 0.15;
    private static final double STALL_SECONDS = 1.0;

    static final class Settings {
        final double p, d, f, maxPower, manualPower, bangBangPower, pdfRangeDegrees;

        Settings(double p, double d, double f, double maxPower, double manualPower,
                 double bangBangPower, double pdfRangeDegrees) {
            this.p = p;
            this.d = d;
            this.f = f;
            this.maxPower = maxPower;
            this.manualPower = manualPower;
            this.bangBangPower = bangBangPower;
            this.pdfRangeDegrees = pdfRangeDegrees;
        }

        boolean valid() {
            return finiteNonnegative(p) && finiteNonnegative(d) && finiteNonnegative(f)
                    && finiteNonnegative(maxPower) && maxPower > 0.0 && maxPower <= 1.0
                    && finiteNonnegative(manualPower) && manualPower <= 1.0
                    && finiteNonnegative(bangBangPower) && bangBangPower > 0.0 && bangBangPower <= 1.0
                    && finiteNonnegative(pdfRangeDegrees);
        }

        private static boolean finiteNonnegative(double value) {
            return Double.isFinite(value) && value >= 0.0;
        }
    }

    private boolean previousX, previousY, previousB, previousLb, previousRb;
    private boolean automatic;
    private double targetDegrees = 90.0;
    private String fault;
    private String state = "Idle";
    private double lastAngle = Double.NaN;
    private double lastTime;
    private double velocity;
    private double power;
    private long progressTicks;
    private double progressTime;
    private double directionStarted;
    private double settledSince = Double.NaN;

    TurretAimController(boolean xInitiallyHeld, boolean yInitiallyHeld, boolean bInitiallyHeld) {
        previousX = xInitiallyHeld;
        previousY = yInitiallyHeld;
        previousB = bInitiallyHeld;
    }

    /** Rebase after the magnet changes zero, without cancelling the requested move. */
    void rebase(double angle, long ticks, double now) {
        lastAngle = angle;
        lastTime = now;
        velocity = 0.0;
        progressTicks = ticks;
        progressTime = now;
        directionStarted = now;
        power = 0.0;
        settledSince = Double.NaN;
    }

    double update(double angle, long ticks, double now, boolean lb, boolean rb,
                  boolean x, boolean y, boolean b, Settings settings) {
        boolean xPressed = x && !previousX;
        boolean yPressed = y && !previousY;
        boolean bPressed = b && !previousB;
        boolean manualPressed = (lb && !previousLb) || (rb && !previousRb);
        previousX = x;
        previousY = y;
        previousB = b;
        previousLb = lb;
        previousRb = rb;
        if (!settings.valid() || !Double.isFinite(angle) || !Double.isFinite(now)) {
            fault = "Invalid control settings or encoder angle";
            automatic = false;
            return stop(ticks, now);
        }

        double dt = now - lastTime;
        if (Double.isNaN(lastAngle) || dt <= 0.0) {
            velocity = 0.0;
        } else {
            // Low-pass derivative on measurement avoids target-change derivative kicks.
            double measuredVelocity = (angle - lastAngle) / dt;
            velocity += dt / (0.05 + dt) * (measuredVelocity - velocity);
        }
        lastAngle = angle;
        lastTime = now;

        // Both bumpers stop and clear faults, with priority over target buttons.
        if (lb && rb) {
            automatic = false;
            fault = null;
            state = "Cancelled / fault cleared";
            settledSince = Double.NaN;
            return stop(ticks, now);
        }
        if (lb || rb) {
            automatic = false;
            settledSince = Double.NaN;
            if (manualPressed) startCommand(angle, ticks, now);
            state = lb ? "Manual positive" : "Manual negative";
        } else if (xPressed || yPressed || bPressed) {
            // Each fresh press replaces the previous target and restarts all motion
            // history. Prefer center if multiple target buttons are pressed together.
            targetDegrees = yPressed ? 0.0 : xPressed ? 90.0 : -90.0;
            startCommand(angle, ticks, now);
            automatic = true;
            state = "Aiming at " + targetDegrees + " degrees";
        }
        // A held input cannot keep retrying a stall or direction fault. Release and
        // press a command again to retry; every attempt still obeys travel limits.
        if (fault != null) {
            automatic = false;
            return stop(ticks, now);
        }

        double requested = 0.0;
        if (lb != rb) {
            requested = lb ? settings.manualPower : -settings.manualPower;
        } else if (automatic) {
            double error = targetDegrees - angle;
            if (Math.abs(error) <= ANGLE_TOLERANCE_DEGREES) {
                if (Double.isNaN(settledSince)) settledSince = now;
                if (now - settledSince >= SETTLE_SECONDS) {
                    automatic = false;
                    state = "Reached " + targetDegrees + " degrees";
                } else {
                    state = "Settling at " + targetDegrees + " degrees";
                }
            } else {
                settledSince = Double.NaN;
                if (Math.abs(error) > settings.pdfRangeDegrees) {
                    state = "Bang-bang to " + targetDegrees + " degrees";
                    requested = settings.bangBangPower * Math.signum(error);
                } else {
                    state = "PDF to " + targetDegrees + " degrees";
                    requested = settings.p * error - settings.d * velocity
                            + settings.f * Math.signum(error);
                }
            }
        }
        requested = Math.max(-settings.maxPower, Math.min(settings.maxPower, requested));

        // Block outward travel at either limit, but allow manual recovery inward.
        if ((angle >= LIMIT_DEGREES && requested > 0.0)
                || (angle <= -LIMIT_DEGREES && requested < 0.0)) {
            automatic = false;
            state = "Travel limit: outward motion blocked";
            return stop(ticks, now);
        }
        if (requested == 0.0) return stop(ticks, now);

        // Monitor both manual and automatic movement. Do not mistake an intentional
        // reversal or a magnet rebase for incorrect encoder direction.
        if (power == 0.0 || Math.signum(requested) != Math.signum(power)) {
            progressTicks = ticks;
            progressTime = now;
            directionStarted = now;
        } else {
            double signedProgress = (ticks - progressTicks) * Math.signum(power);
            if (signedProgress < -4.0 && now - directionStarted >= 0.15) {
                fault = "Encoder direction disagrees with power: check ENCODER_SIGN";
            } else if (signedProgress >= 1.0 || now - directionStarted < 0.15) {
                progressTicks = ticks;
                progressTime = now;
            } else if (now - progressTime >= STALL_SECONDS) {
                fault = "No encoder progress for 1 second: check encoder / wiring";
            }
            if (fault != null) {
                automatic = false;
                return stop(ticks, now);
            }
        }
        power = requested;
        return power;
    }

    private double stop(long ticks, double now) {
        power = 0.0;
        progressTicks = ticks;
        progressTime = now;
        return 0.0;
    }

    private void startCommand(double angle, long ticks, double now) {
        fault = null;
        rebase(angle, ticks, now);
    }

    String getState() {
        return fault == null ? state : "FAULT: " + fault + " (release/re-press command to retry)";
    }

    double getTargetDegrees() {
        return targetDegrees;
    }

    double getVelocity() {
        return velocity;
    }
}
