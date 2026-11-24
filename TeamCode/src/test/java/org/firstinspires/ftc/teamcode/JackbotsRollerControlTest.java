package org.firstinspires.ftc.teamcode;

/** Roller toggle and held-outtake interaction without hardware. */
public final class JackbotsRollerControlTest {
    private static int checks;

    private static void expect(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    public static void main(String[] args) {
        JackbotsTeleOp.RollerControl rollers = new JackbotsTeleOp.RollerControl(false);
        expect(rollers.update(false, false) == 0, "Rollers start off");
        expect(rollers.update(true, false) == -1, "LB press starts intake");
        expect(rollers.update(true, false) == -1, "Held LB does not toggle repeatedly");
        expect(rollers.update(false, false) == -1, "Intake continues after LB release");
        expect(rollers.update(false, true) == 1, "Held RB overrides latched intake");
        expect(rollers.update(false, true) == 1, "Outtake continues while RB is held");
        expect(rollers.update(false, false) == -1, "RB release restores latched intake");
        expect(rollers.update(true, false) == 0, "Next LB press stops intake");
        expect(rollers.update(false, true) == 1, "RB also outtakes when intake is off");
        expect(rollers.update(false, false) == 0, "RB release stops when intake is off");
        expect(rollers.update(true, true) == 1, "RB wins when LB toggles intake on simultaneously");
        expect(rollers.update(false, false) == -1, "Simultaneous press still records intake toggle");
        expect(rollers.update(true, true) == 1, "RB wins when LB toggles intake off simultaneously");
        expect(rollers.update(false, false) == 0, "Intake stays off after outtake release");

        rollers = new JackbotsTeleOp.RollerControl(true);
        expect(rollers.update(true, false) == 0, "LB held at START requires a fresh press");
        rollers.update(false, false);
        expect(rollers.update(true, false) == -1, "Fresh LB press after START enables intake");
        System.out.println("Passed " + checks + " roller control checks.");
    }
}
