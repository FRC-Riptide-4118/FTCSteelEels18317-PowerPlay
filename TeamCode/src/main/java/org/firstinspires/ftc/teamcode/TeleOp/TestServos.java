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

        Servo armRight  = hardwareMap.servo.get("arm_right");
        Servo armLeft   = hardwareMap.servo.get("arm_left");
        Servo wrist     = hardwareMap.servo.get("wrist");
        Servo align     = hardwareMap.servo.get("alignment");
        Servo grip      = hardwareMap.servo.get("gripper");

        waitForStart();

        while(opModeIsActive()) {
            armRight.setPosition    (gamepad1.a             ? 0.5 : 0.4);
            armLeft.setPosition     (gamepad1.b             ? 0.5 : 0.6);
            wrist.setPosition       (gamepad1.x             ? 0.5 : 0.6);
            align.setPosition       (gamepad1.left_bumper   ? 0.5 : 0.6);
            grip.setPosition        (gamepad1.right_bumper  ? 0.5 : 0.6);


            m_telemetry.addData("arm1",     armRight.getPosition());
            m_telemetry.addData("arm2",     armLeft.getPosition());
            m_telemetry.addData("wrist",    wrist.getPosition());
            m_telemetry.addData("align",    align.getPosition());
            m_telemetry.addData("grip",     grip.getPosition());

            m_telemetry.update();
        }
    }

}
