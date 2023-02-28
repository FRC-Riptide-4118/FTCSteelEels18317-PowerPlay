package org.firstinspires.ftc.teamcode.BlakeStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "(Blake) Test Single Motor", group = "Blake")
public class TestSingleMotor extends LinearOpMode {

    DcMotor testMotor;
    String testMotorName = "front_left_drive";

    public void runOpMode()
    {
        testMotor = hardwareMap.dcMotor.get(testMotorName);


        waitForStart();


        while(opModeIsActive())
        {
            testMotor.setPower(gamepad1.left_stick_y);

            telemetry.addData("Gamepad input", gamepad1.left_stick_y);
            telemetry.addData("Test Motor Power", testMotor.getPower());
            telemetry.addData("Test Motor Position", testMotor.getCurrentPosition());
            telemetry.update();
        }
    }

}
