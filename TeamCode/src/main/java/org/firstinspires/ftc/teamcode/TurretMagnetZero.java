package org.firstinspires.ftc.teamcode;

/** Software zero at the negative-side entry into the magnet's detection zone. */
final class TurretMagnetZero {
    private boolean sampled;
    private boolean previousDetected;
    private int previousTicks;
    private int lastMovementDirection;
    private int zeroTicks;

    void update(int ticks, boolean detected) {
        if (!sampled) {
            // Startup is only a provisional reference, not a magnet entry event.
            zeroTicks = ticks;
            sampled = true;
        } else {
            long movement = (long) ticks - previousTicks;
            if (movement != 0) lastMovementDirection = Long.signum(movement);
            // Increasing encoder angle means approaching from the negative side.
            // Use motion, not the provisional angle or the commanded servo power:
            // the physical zero is unknown until the first qualifying entry.
            if (detected && !previousDetected && lastMovementDirection > 0) {
                zeroTicks = ticks;
            }
        }
        previousTicks = ticks;
        previousDetected = detected;
    }

    long zeroedTicks() {
        return (long) previousTicks - zeroTicks;
    }
}
