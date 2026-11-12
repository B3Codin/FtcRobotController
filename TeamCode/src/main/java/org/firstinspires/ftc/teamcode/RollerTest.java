package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Roller Test", group = "Test")
public class RollerTest extends LinearOpMode {
    // Change to -1.0 if LB rolls outward on your robot.
    private static final double INTAKE_POWER = 1.0;

    @Override
    public void runOpMode() {
        DcMotor rollLeft = hardwareMap.get(DcMotor.class, "rollLeft");
        DcMotor rollRight = hardwareMap.get(DcMotor.class, "rollRight");
        Servo intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        Servo intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");

        rollLeft.setPower(0.0);
        rollRight.setPower(0.0);
        rollLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        rollRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rollLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rollRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Gamepad 1: hold LB to roll in | hold RB to roll out");
        telemetry.addLine("Release both or hold both bumpers to stop.");
        telemetry.addLine("X: intakeRight = 0.51 | intakeLeft = 0.66");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                double power = 0.0;
                String rollerState = "Stopped";
                if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                    power = INTAKE_POWER;
                    rollerState = "In";
                } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                    power = -INTAKE_POWER;
                    rollerState = "Out";
                }

                // The reversed right motor spins opposite the left motor.
                rollLeft.setPower(power);
                rollRight.setPower(power);

                if (gamepad1.x) {
                    intakeRight.setPosition(0.51);
                    intakeLeft.setPosition(0.66);
                }

                telemetry.addData("Rollers", rollerState);
                telemetry.addData("Power", "%.2f", power);
                telemetry.addData("intakeRight commanded position", "%.2f",
                        intakeRight.getPosition());
                telemetry.addData("intakeLeft commanded position", "%.2f",
                        intakeLeft.getPosition());
                telemetry.update();
                sleep(20);
            }
        } finally {
            rollLeft.setPower(0.0);
            rollRight.setPower(0.0);
        }
    }
}
