package org.firstinspires.ftc.teamcode;

/** Simulates reset/calibration/readback sequences without robot hardware. */
public final class JackbotsStartingPoseTest {
    private static int checks;

    private static void expect(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    public static void main(String[] args) {
        JackbotsTeleOp.StartingPose pose = new JackbotsTeleOp.StartingPose();
        expect(!pose.shouldApply(true, 0), "Stale READY right after reset cannot initialize pose");
        expect(!pose.shouldApply(true, 0.499), "Wait through reset grace period");
        expect(!pose.shouldApply(false, 0.6), "Calibration longer than grace period still blocks initialization");
        expect(pose.shouldApply(true, 0.7), "Initialize when calibration finishes");
        pose.confirm(true, 0, 0, 0);
        expect(pose.shouldApply(true, 0.8), "Cached zero readback does not count as success");
        pose.confirm(false, 198, -217.4, 0);
        expect(pose.shouldApply(true, 0.9), "Bad read with correct numbers does not count as success");
        pose.confirm(true, 198, -217.4, 10);
        expect(pose.shouldApply(true, 1.0), "Heading must also match starting pose");
        pose.confirm(true, 198, -217.3999939, 0);
        expect(!pose.shouldApply(true, 1.1), "Verified pose ends initialization, allowing float precision");
        pose.confirm(true, 0, 0, 0);
        expect(!pose.shouldApply(true, 2), "A reset must not trigger another initial-position write");
        pose.confirm(false, 0, 0, 0);
        expect(!pose.shouldApply(true, 3), "Later status changes must not reinitialize during driving");
        System.out.println("Passed " + checks + " starting pose checks.");
    }
}
