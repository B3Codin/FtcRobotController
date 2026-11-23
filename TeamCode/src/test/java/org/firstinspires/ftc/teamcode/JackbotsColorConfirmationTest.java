package org.firstinspires.ftc.teamcode;

/** Standalone continuous-detection timing regression; no hardware or sleeps. */
public final class JackbotsColorConfirmationTest {
    private static int checks;

    private static void expect(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    public static void main(String[] args) {
        JackbotsTeleOp.ColorConfirmation color = new JackbotsTeleOp.ColorConfirmation();
        expect(!color.update(false, 100), "Time spent in INIT/without color does not count");
        expect(!color.update(true, 101), "First detection starts timer without stopping intake");
        expect(!color.update(true, 101.999), "Less than a full second cannot confirm");
        expect(color.update(true, 102), "Exactly one continuous second confirms");
        expect(color.update(true, 103), "Continued color keeps confirmation active");
        expect(!color.update(false, 103.01), "Clear reading immediately clears confirmation");
        expect(!color.update(true, 104), "Detection after clearing requires a new full second");
        expect(!color.update(true, 104.9), "Earlier confirmed detection contributes no time");
        expect(!color.update(false, 104.95), "Brief loss resets a pending detection");
        expect(!color.update(true, 105), "Second pulse begins a new interval");
        expect(!color.update(true, 105.5), "Separate pulses cannot accumulate to one second");
        expect(color.update(true, 106), "New uninterrupted interval can confirm again");
        System.out.println("Passed " + checks + " color confirmation checks.");
    }
}
