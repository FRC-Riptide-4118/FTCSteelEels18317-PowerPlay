package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.GripperGripCone;
import org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(group = "Test")
@Config
public class TestServos extends LinearOpMode {
    public static double uprightPos1 = 0.5;
    public static double uprightPos2 = 0.6;

    @Override
    public void runOpMode() {
        Telemetry m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo upright   = hardwareMap.servo.get("upright");

        waitForStart();

        while(opModeIsActive()) {
            upright.setPosition(gamepad1.a ? uprightPos1 : uprightPos2);

            m_telemetry.addData("upright", upright.getPosition());
            m_telemetry.update();
        }
    }

}
