package org.firstinspires.ftc.teamcode;

/** Hardware-independent turret control. Positive power must increase signed encoder ticks. */
final class TurretAimController {
    static final double TARGET_DEGREES = 90.0;
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

    private boolean previousX;
    private boolean automatic;
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

    TurretAimController(boolean xInitiallyHeld) {
        previousX = xInitiallyHeld;
    }

    /** Rebase after the magnet changes zero, without cancelling the requested move. */
    void rebase(double angle, long ticks, double now) {
        lastAngle = angle;
        lastTime = now;
        velocity = 0.0;
        progressTicks = ticks;
        progressTime = now;
        settledSince = Double.NaN;
    }

    double update(double angle, long ticks, double now, boolean lb, boolean rb,
                  boolean x, boolean b, Settings settings) {
        boolean xPressed = x && !previousX;
        previousX = x;
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

        // B has priority over all commands. A released/re-pressed X is required to restart.
        if (b) {
            automatic = false;
            fault = null;
            state = "Cancelled / fault cleared";
            settledSince = Double.NaN;
            return stop(ticks, now);
        }
        if (fault != null) {
            automatic = false;
            return stop(ticks, now);
        }
        if (lb || rb) {
            automatic = false;
            settledSince = Double.NaN;
            state = lb && rb ? "Both bumpers: stopped" : "Manual";
        } else if (xPressed) {
            automatic = true;
            settledSince = Double.NaN;
            state = "Aiming at +90 degrees";
        }

        double requested = 0.0;
        if (lb != rb) {
            requested = lb ? settings.manualPower : -settings.manualPower;
        } else if (automatic) {
            double error = TARGET_DEGREES - angle;
            if (Math.abs(error) <= ANGLE_TOLERANCE_DEGREES) {
                if (Double.isNaN(settledSince)) settledSince = now;
                if (now - settledSince >= SETTLE_SECONDS) {
                    automatic = false;
                    state = "Reached +90 degrees";
                } else {
                    state = "Settling at +90 degrees";
                }
            } else {
                settledSince = Double.NaN;
                if (Math.abs(error) > settings.pdfRangeDegrees) {
                    state = "Bang-bang to +90 degrees";
                    requested = settings.bangBangPower * Math.signum(error);
                } else {
                    state = "PDF to +90 degrees";
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

    String getState() {
        return fault == null ? state : "FAULT: " + fault + " (B clears)";
    }

    double getVelocity() {
        return velocity;
    }
}
