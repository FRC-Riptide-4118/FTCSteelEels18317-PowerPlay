package org.firstinspires.ftc.teamcode.BlakeStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BlakeTestDashboardTelemetry extends LinearOpMode
{
    // Hardware class
    BlakeRobotHardware hardware;

    // FTC Dashboard telemetry
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Instantiate and initialize hardware
        hardware = new BlakeRobotHardware(hardwareMap);
        hardware.init();

        // FTC Dashboard telemetry
        packet.put("Status", "Ready!");
        dashboard.sendTelemetryPacket(packet);

        // Driver station telemetry
        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {

            // FTC Dashboard telemetry
            packet.put("Status", "Running!");
            dashboard.sendTelemetryPacket(packet);

            // Driver station telemetry
            telemetry.addLine("Running");
            telemetry.addData("Front left drive pos", hardware.frontLeft.getCurrentPosition());
            telemetry.addData("Front right drive pos", hardware.frontRight.getCurrentPosition());
            telemetry.addData("Rear left drive pos", hardware.rearLeft.getCurrentPosition());
            telemetry.addData("Rear right drive pos", hardware.rearRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
