package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorTesting")
public class MotorTesting extends LinearOpMode {

    private DcMotor FLeft;




    @Override
    public void runOpMode(){

        FLeft.setPower(1.0);

    }
}


