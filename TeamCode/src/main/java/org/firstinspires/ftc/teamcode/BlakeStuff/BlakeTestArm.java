package org.firstinspires.ftc.teamcode.BlakeStuff;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EelverHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
@Disabled
@TeleOp(group = "Blake")
public class BlakeTestArm extends LinearOpMode {

    EelverHardware hardware;

    public void runOpMode()
    {
        hardware = new EelverHardware();

        hardware.init(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a) hardware.armToStart();
            else if(gamepad1.x) hardware.armToLow();
            else if(gamepad1.y) hardware.armToMedium();
            else if(gamepad1.b) hardware.armToHigh();

            telemetry.addData("arm1 pos", hardware.arm1.getPosition());
            telemetry.addData("arm2 pos", hardware.arm2.getPosition());
            telemetry.update();

        }

    }
}
