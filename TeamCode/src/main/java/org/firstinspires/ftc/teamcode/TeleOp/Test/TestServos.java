package org.firstinspires.ftc.teamcode.TeleOp.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@TeleOp(group = "Test")
@Config
public class TestServos extends LinearOpMode {
    public static double armRightPos1 = 0.6;
    public static double armRightPos2 = 0.5;
    public static double armLeftPos1 = 0.6;
    public static double armLeftPos2 = 0.5;

    public static double uprightPos1 = 0.5;
    public static double uprightPos2 = 0.6;

    @Override
    public void runOpMode() {
        Telemetry m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo armRight  = hardwareMap.servo.get("arm_right");
        Servo armLeft   = hardwareMap.servo.get("arm_left");
        Servo upright   = hardwareMap.servo.get("upright");

        waitForStart();

        while(opModeIsActive()) {
            armRight.setPosition    (gamepad1.a             ? armRightPos1 : armRightPos2);
            armLeft.setPosition     (gamepad1.b             ? armLeftPos1 : armLeftPos2);
            upright.setPosition     (gamepad1.y             ? uprightPos1 : uprightPos2);

            m_telemetry.addData("armRight", armRight.getPosition());
            m_telemetry.addData("armLeft",  armLeft.getPosition());
            m_telemetry.addData("upright",  upright.getPosition());
            m_telemetry.update();
        }
    }

}
