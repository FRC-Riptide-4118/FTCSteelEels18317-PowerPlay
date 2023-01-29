package org.firstinspires.ftc.teamcode.BlakeStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "(Blake) Test Hardware Class", group = "Blake")
public class BlakeTestHardwareClass extends LinearOpMode
{
    // Hardware class
    BlakeRobotHardware hardware; // Call this "robot", "hardware", or whatever else makes sense.

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Instantiate the hardware class
        hardware = new BlakeRobotHardware(hardwareMap);
        hardware.init();


        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {
            // Some simple controls
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = -gamepad1.right_stick_x;

            hardware.setMecanumPower(drive, strafe, twist, 1.0);


            // Telemetry
            telemetry.addLine("Drive Motors:");
            telemetry.addData("Front left power: ",     hardware.frontLeft.getPower());
            telemetry.addData("Front right power: ",    hardware.frontRight.getPower());
            telemetry.addData("Rear left power: ",      hardware.rearLeft.getPower());
            telemetry.addData("Rear right power: ",     hardware.rearRight.getPower());
            telemetry.update();

        }
    }
}
