package org.firstinspires.ftc.teamcode;

/** Shooting release, continuous detection and manual intake regression checks. */
public final class JackbotsIntakeControlTest {
    private static int checks;

    private static void expect(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    public static void main(String[] args) {
        JackbotsTeleOp.IntakeControl intake = new JackbotsTeleOp.IntakeControl(false);
        intake.update(false, 0, false, false);
        expect(!intake.intakeUp && !intake.colorConfirmed, "Start down with purple indicators");
        intake.update(true, 1, false, false);
        intake.update(true, 1.49, false, false);
        expect(!intake.intakeUp && !intake.colorConfirmed, "Short detection cannot raise or turn red");
        intake.update(false, 1.5, false, false);
        intake.update(true, 2, false, false);
        intake.update(true, 2.5, false, false);
        expect(intake.intakeUp && intake.colorConfirmed, "Half a continuous second raises and turns red");
        intake.update(true, 3, true, false);
        expect(intake.intakeUp, "Holding shoot does not lower the intake");
        intake.update(true, 3.1, false, true);
        expect(!intake.intakeUp && !intake.colorConfirmed,
                "Y release lowers intake and clears old confirmation even with B and color held");
        intake.update(true, 3.2, false, true);
        expect(!intake.intakeUp, "Old confirmation cannot raise intake on the next loop");
        intake.update(true, 3.69, false, false);
        expect(!intake.intakeUp, "A full new interval is required after shooting");
        intake.update(true, 3.71, false, false);
        expect(intake.intakeUp && intake.colorConfirmed, "A new confirmed ball raises the intake");
        intake.update(false, 4, false, false);
        expect(intake.intakeUp && !intake.colorConfirmed, "Clear sensor turns purple without lowering");
        intake.update(false, 4.1, false, true);
        expect(!intake.intakeUp, "B still toggles down");
        intake.update(false, 4.2, false, true);
        expect(!intake.intakeUp, "Holding B does not toggle repeatedly");
        intake.update(false, 5, true, false);
        intake.update(false, 5.1, false, false);
        expect(!intake.intakeUp, "Releasing Y when already down keeps it down");

        intake = new JackbotsTeleOp.IntakeControl(true);
        intake.update(true, 0, true, false);
        intake.update(true, 0.5, true, false);
        intake.update(true, 0.6, false, false);
        expect(!intake.intakeUp && !intake.colorConfirmed, "Y held at START still handles its release");
        System.out.println("Passed " + checks + " intake control checks.");
    }
}
