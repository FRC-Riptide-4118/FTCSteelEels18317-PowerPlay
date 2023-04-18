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
//        Servo wrist     = hardwareMap.servo.get("wrist");
//        Servo align     = hardwareMap.servo.get("alignment");
//        Servo grip      = hardwareMap.servo.get("gripper");

        waitForStart();

        while(opModeIsActive()) {
            armRight.setPosition    (gamepad1.a             ? armRightPos1 : armRightPos2);
            armLeft.setPosition     (gamepad1.b             ? armLeftPos1 : armLeftPos2);
            upright.setPosition     (gamepad1.y             ? uprightPos1 : uprightPos2);

//            wrist.setPosition       (gamepad1.x             ? 0.5 : 0.6);
//            align.setPosition       (gamepad1.left_bumper   ? 0.5 : 0.6);
//            grip.setPosition        (gamepad1.right_bumper  ? 0.5 : 0.6);


            m_telemetry.addData("armRight", armRight.getPosition());
            m_telemetry.addData("armLeft",  armLeft.getPosition());
            m_telemetry.addData("upright",  upright.getPosition());
//            m_telemetry.addData("wrist",    wrist.getPosition());
//            m_telemetry.addData("align",    align.getPosition());
//            m_telemetry.addData("grip",     grip.getPosition());

            m_telemetry.update();
        }
    }

}
