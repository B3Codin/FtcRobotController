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
        expect(rollers.update(false, false, false) == 0, "Rollers start off");
        expect(rollers.update(true, false, false) == -1, "LB press starts intake");
        expect(rollers.update(true, false, false) == -1, "Held LB does not toggle repeatedly");
        expect(rollers.update(false, false, false) == -1, "Intake continues after LB release");
        expect(rollers.update(false, true, false) == 1, "Held RB overrides latched intake");
        expect(rollers.update(false, true, false) == 1, "Outtake continues while RB is held");
        expect(rollers.update(false, false, false) == -1, "RB release restores latched intake");
        expect(rollers.update(true, false, false) == 0, "Next LB press stops intake");
        expect(rollers.update(false, true, false) == 1, "RB also outtakes when intake is off");
        expect(rollers.update(false, false, false) == 0, "RB release stops when intake is off");
        expect(rollers.update(true, true, false) == 1, "RB wins when LB toggles intake on simultaneously");
        expect(rollers.update(false, false, false) == -1, "Simultaneous press still records intake toggle");
        expect(rollers.update(true, true, false) == 1, "RB wins when LB toggles intake off simultaneously");
        expect(rollers.update(false, false, false) == 0, "Intake stays off after outtake release");

        rollers = new JackbotsTeleOp.RollerControl(true);
        expect(rollers.update(true, false, false) == 0, "LB held at START requires a fresh press");
        rollers.update(false, false, false);
        expect(rollers.update(true, false, false) == -1, "Fresh LB press after START enables intake");

        JackbotsTeleOp.IntakeControl intake = new JackbotsTeleOp.IntakeControl(false);
        intake.update(true, 1, false, false);
        intake.update(true, 1.249, false, false);
        expect(rollers.update(false, false, intake.colorConfirmed) == -1,
                "Intake stays full power before the 0.25-second threshold");
        intake.update(true, 1.25, false, false);
        expect(rollers.update(false, false, intake.colorConfirmed) == -0.5 && intake.intakeUp,
                "Confirmed ball raises intake and reduces roller power to half");
        expect(rollers.update(false, true, intake.colorConfirmed) == 1,
                "Outtake stays full power with a confirmed ball");
        expect(rollers.update(false, false, intake.colorConfirmed) == -0.5,
                "Releasing outtake resumes the reduced intake power");
        expect(rollers.update(true, false, intake.colorConfirmed) == 0,
                "Confirmation cannot turn on rollers that were toggled off");
        rollers.update(false, false, intake.colorConfirmed);
        expect(rollers.update(true, false, intake.colorConfirmed) == -0.5,
                "Toggling intake back on respects the confirmed-ball limit");
        intake.update(false, 1.3, false, false);
        expect(rollers.update(false, false, intake.colorConfirmed) == -1,
                "Clearing detection restores normal intake power");
        System.out.println("Passed " + checks + " roller control checks.");
    }
}
