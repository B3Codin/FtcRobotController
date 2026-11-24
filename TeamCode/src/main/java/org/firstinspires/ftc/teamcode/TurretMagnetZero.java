package org.firstinspires.ftc.teamcode;

/** Software angle reference at the negative-side entry into the magnet's detection zone. */
final class TurretMagnetZero {
    static final double TICKS_PER_DEGREE = 8192.0 * (90.0 / 24.0) / 360.0;
    private boolean sampled;
    private boolean previousDetected;
    private int previousTicks;
    private int lastMovementDirection;
    private int zeroTicks;
    private int initialTicks;
    private double referenceAngleDegrees;

    boolean update(int ticks, boolean detected) {
        return update(ticks, detected, 0.0);
    }

    /** Returns true only when this sample applies a magnet correction. */
    boolean update(int ticks, boolean detected, double magnetAngleDegrees) {
        boolean corrected = false;
        if (!sampled) {
            // Startup is only a provisional reference, not a magnet entry event.
            zeroTicks = ticks;
            initialTicks = ticks;
            sampled = true;
        } else {
            long movement = (long) ticks - previousTicks;
            if (movement != 0) lastMovementDirection = Long.signum(movement);
            // Increasing encoder angle means approaching from the negative side.
            // Use motion, not the provisional angle or the commanded servo power:
            // the physical zero is unknown until the first qualifying entry.
            corrected = detected && !previousDetected && lastMovementDirection > 0
                    && Double.isFinite(magnetAngleDegrees);
        }
        previousTicks = ticks;
        previousDetected = detected;
        if (corrected) setAngleDegrees(magnetAngleDegrees);
        return corrected;
    }

    /** Change the corrected reference only; the initial reference always stays fixed. */
    void setAngleDegrees(double angleDegrees) {
        zeroTicks = previousTicks;
        referenceAngleDegrees = angleDegrees;
    }

    long zeroedTicks() {
        return (long) previousTicks - zeroTicks;
    }

    double angleDegrees() {
        return zeroedTicks() / TICKS_PER_DEGREE + referenceAngleDegrees;
    }

    double unzeroedAngleDegrees() {
        return ((long) previousTicks - initialTicks) / TICKS_PER_DEGREE;
    }
}
