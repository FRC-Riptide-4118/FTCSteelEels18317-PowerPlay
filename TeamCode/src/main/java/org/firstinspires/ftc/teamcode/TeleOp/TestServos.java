package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(group = "Test")
@Config
public class TestServos extends LinearOpMode {
    public static double servo1Pos = 0.0;
    public static double servo2Pos = 0.0;

    @Override
    public void runOpMode() {
        Telemetry m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo servo1 = hardwareMap.servo.get("intake_left");
        Servo servo2 = hardwareMap.servo.get("intake_right");

        waitForStart();

        while(opModeIsActive()) {
            servo1.setPosition(servo1Pos);
            servo2.setPosition(servo2Pos);


            m_telemetry.addData("intake left pos",  servo1.getPosition());
            m_telemetry.addData("intake right pos", servo2.getPosition());
            m_telemetry.update();
        }
    }

}
