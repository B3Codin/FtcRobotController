package org.firstinspires.ftc.teamcode;

/** Hardware-independent checks of magnet entry direction and continuous tracking. */
public final class TurretMagnetZeroTest {
    private static int checks;

    private static void expect(long expected, TurretMagnetZero zero, String message) {
        checks++;
        if (zero.zeroedTicks() != expected) {
            throw new AssertionError(message + ": expected " + expected + ", got " + zero.zeroedTicks());
        }
    }

    public static void main(String[] args) {
        TurretMagnetZero zero = new TurretMagnetZero();
        zero.update(1000, false);
        zero.update(800, false);
        zero.update(900, true);
        expect(0, zero, "Increasing angle into the magnet zeros once");
        zero.update(950, true);
        expect(50, zero, "Tracking continues while magnet remains detected");
        zero.update(925, true);
        expect(25, zero, "Reversing inside the detection zone does not zero again");
        zero.update(1100, false);
        zero.update(1000, true);
        expect(100, zero, "Entry from the positive side preserves zero");
        zero.update(1025, true);
        expect(125, zero, "Reversing after positive-side entry cannot trigger zero");
        zero.update(800, false);
        zero.update(910, true);
        expect(0, zero, "A later negative-side entry corrects zero again");
        zero.update(930, true);
        expect(20, zero, "Tracking resumes after repeated calibration");

        TurretMagnetZero startupDetected = new TurretMagnetZero();
        startupDetected.update(500, true);
        startupDetected.update(520, true);
        expect(20, startupDetected, "Starting on magnet does not hold angle at zero");
        startupDetected.update(480, false);
        startupDetected.update(500, true);
        expect(0, startupDetected, "Startup on magnet still permits a later valid entry");

        TurretMagnetZero stationary = new TurretMagnetZero();
        stationary.update(100, false);
        stationary.update(100, true);
        stationary.update(110, true);
        expect(10, stationary, "No motion at startup gives no approach direction");

        TurretMagnetZero delayedDetection = new TurretMagnetZero();
        delayedDetection.update(100, false);
        delayedDetection.update(120, false);
        delayedDetection.update(120, true);
        expect(0, delayedDetection, "Retain approach direction across a stationary sample");
        delayedDetection.update(130, true);
        expect(10, delayedDetection, "Raw startup count does not define physical magnet side");
        System.out.println("Passed " + checks + " magnet zero checks.");
    }
}
