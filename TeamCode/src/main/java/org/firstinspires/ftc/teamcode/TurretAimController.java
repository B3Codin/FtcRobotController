package org.firstinspires.ftc.teamcode;

/** Hardware-independent turret control. Positive power must increase signed encoder ticks. */
final class TurretAimController {
    static final double LIMIT_DEGREES = 180.0;
    // Stop automatic motion within +/-2 degrees, including both boundaries.
    private static final double ANGLE_TOLERANCE_DEGREES = 2.0;
    private static final double SETTLE_SECONDS = 0.15;
    private static final double STALL_SECONDS = 1.0;

    static final class Settings {
        final double p, i, d, f, minPower, maxPower, manualPower, bangBangPower, pdfRangeDegrees;

        Settings(double p, double d, double f, double maxPower, double manualPower,
                 double bangBangPower, double pdfRangeDegrees) {
            this(p, 0.0, d, f, 0.0, maxPower, manualPower, bangBangPower, pdfRangeDegrees);
        }

        Settings(double p, double i, double d, double f, double minPower, double maxPower,
                 double manualPower, double bangBangPower, double pdfRangeDegrees) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
            this.minPower = minPower;
            this.maxPower = maxPower;
            this.manualPower = manualPower;
            this.bangBangPower = bangBangPower;
            this.pdfRangeDegrees = pdfRangeDegrees;
        }

        boolean valid() {
            return finiteNonnegative(p) && finiteNonnegative(i) && finiteNonnegative(d) && finiteNonnegative(f)
                    && finiteNonnegative(minPower) && minPower <= maxPower
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
    private boolean previousTracking;
    private double targetDegrees = 90.0;
    private String fault;
    private String state = "Idle";
    private double lastAngle = Double.NaN;
    private double lastTime;
    private double velocity;
    private double power;
    // Store the I contribution in power units; cap it and discard it on mode changes.
    private double integralPower;
    private double previousError;
    private double previousIGain = Double.NaN;
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
        integralPower = 0.0;
        previousError = 0.0;
        settledSince = Double.NaN;
    }

    double update(double angle, long ticks, double now, boolean lb, boolean rb,
                  boolean x, boolean y, boolean b, Settings settings) {
        return update(angle, ticks, now, lb, rb, x, y, b, settings, false, 0.0);
    }

    /** Continuous targets retain fault history between samples. Toggle off/on to retry. */
    double update(double angle, long ticks, double now, boolean lb, boolean rb,
                  boolean x, boolean y, boolean b, Settings settings,
                  boolean tracking, double trackingTarget) {
        boolean trackingStarted = tracking && !previousTracking;
        if (!tracking && previousTracking) {
            automatic = false;
            state = "Tracking stopped";
        }
        previousTracking = tracking;
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
        // Live edits to I cannot revive accumulated error from a different gain.
        if (settings.i != previousIGain) integralPower = 0.0;
        previousIGain = settings.i;
        integralPower = Math.max(-settings.maxPower, Math.min(settings.maxPower, integralPower));

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
        } else if (tracking) {
            if (trackingStarted) startCommand(angle, ticks, now);
            if (!Double.isFinite(trackingTarget) || Math.abs(trackingTarget) > LIMIT_DEGREES) {
                fault = "Tracking requires calibrated zero, valid pose and a distinct field point";
            } else if (!trackingStarted && Math.abs(trackingTarget - targetDegrees) > 180.0) {
                fault = "Target crossed rear travel boundary; turn chassis and toggle tracking";
            } else {
                targetDegrees = trackingTarget;
                automatic = true;
            }
        }
        // A held input cannot keep retrying a stall or direction fault. Release and
        // press a command again to retry; every attempt still obeys travel limits.
        if (fault != null) {
            automatic = false;
            return stop(ticks, now);
        }

        double requested = 0.0;
        if (lb != rb) {
            integralPower = 0.0;
            requested = lb ? settings.manualPower : -settings.manualPower;
        } else if (automatic) {
            double error = targetDegrees - angle;
            if (Math.abs(error) <= ANGLE_TOLERANCE_DEGREES) {
                if (Double.isNaN(settledSince)) settledSince = now;
                if (tracking) {
                    state = "Tracking field point / aligned";
                } else if (now - settledSince >= SETTLE_SECONDS) {
                    automatic = false;
                    state = "Reached " + targetDegrees + " degrees";
                } else {
                    state = "Settling at " + targetDegrees + " degrees";
                }
            } else {
                settledSince = Double.NaN;
                if (Math.abs(error) > settings.pdfRangeDegrees) {
                    integralPower = 0.0;
                    state = "Bang-bang to " + targetDegrees + " degrees";
                    requested = settings.bangBangPower * Math.signum(error);
                } else {
                    state = "PIDF to " + targetDegrees + " degrees";
                    if (Math.signum(error) != Math.signum(previousError)) integralPower = 0.0;
                    double basePower = settings.p * error - settings.d * velocity
                            + settings.f * Math.signum(error);
                    if (dt > 0.0 && settings.i > 0.0) {
                        double nextIntegral = Math.max(-settings.maxPower, Math.min(settings.maxPower,
                                integralPower + settings.i * error * dt));
                        double candidate = basePower + nextIntegral;
                        // Integrate only if the output fits, or the error would unwind saturation.
                        if (Math.abs(candidate) <= settings.maxPower
                                || Math.signum(error) != Math.signum(candidate)) {
                            integralPower = nextIntegral;
                        }
                    }
                    requested = basePower + integralPower;
                }
                previousError = error;
            }
        }
        if (!Double.isFinite(requested)) {
            fault = "Nonfinite turret PIDF output; check gains";
            automatic = false;
            return stop(ticks, now);
        }
        // Zero stays zero at the target, when cancelled, or when faulted.
        if (requested != 0.0) {
            requested = Math.copySign(Math.max(settings.minPower, Math.abs(requested)), requested);
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
                fault = "No turret movement for 1 second: check command power, mechanism and encoder";
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
        integralPower = 0.0;
        previousError = 0.0;
        progressTicks = ticks;
        progressTime = now;
        return 0.0;
    }

    void cancel(double angle, long ticks, double now) {
        automatic = false;
        previousTracking = false;
        state = "Cancelled";
        rebase(angle, ticks, now);
    }

    /** Heading is CCW-positive; measured turret angles are clockwise-positive. */
    static double fieldTargetDegrees(double robotX, double robotY, double heading,
                                     double targetX, double targetY,
                                     double pivotForward, double pivotLeft) {
        return fieldTargetDegrees(robotX, robotY, heading, targetX, targetY,
                pivotForward, pivotLeft, 0.0);
    }

    static double fieldTargetDegrees(double robotX, double robotY, double heading,
                                     double targetX, double targetY,
                                     double pivotForward, double pivotLeft, double clockwiseTrimDegrees) {
        double pivotX = robotX + pivotForward * Math.cos(heading) - pivotLeft * Math.sin(heading);
        double pivotY = robotY + pivotForward * Math.sin(heading) + pivotLeft * Math.cos(heading);
        double dx = targetX - pivotX;
        double dy = targetY - pivotY;
        if (!Double.isFinite(dx) || !Double.isFinite(dy) || !Double.isFinite(heading)
                || !Double.isFinite(clockwiseTrimDegrees)) {
            return Double.NaN;
        }
        if (Math.hypot(dx, dy) < 0.05) return Double.NaN;
        double clockwiseAngle = heading - Math.atan2(dy, dx) + Math.toRadians(clockwiseTrimDegrees);
        return Math.toDegrees(Math.atan2(Math.sin(clockwiseAngle), Math.cos(clockwiseAngle)));
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
