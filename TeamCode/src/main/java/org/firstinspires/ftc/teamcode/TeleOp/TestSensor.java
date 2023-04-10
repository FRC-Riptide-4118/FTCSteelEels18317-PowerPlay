package org.firstinspires.ftc.teamcode.TeleOp;

import android.hardware.Sensor;
import android.provider.CalendarContract;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(group = "Test")
@Config
public class TestSensor extends LinearOpMode {

    @Override
    public void runOpMode() {
        Telemetry m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ColorSensor color = hardwareMap.colorSensor.get("color_cone");

        waitForStart();

        while(opModeIsActive()) {
            double r = color.red(), g = color.green(), b = color.blue();

            double mag = Math.sqrt((r*r) + (g*g) + (b*b));

            if(gamepad1.x) color.enableLed(true);
            else color.enableLed(false);



            m_telemetry.addData("r", r);
            m_telemetry.addData("g", g);
            m_telemetry.addData("b", b);
            m_telemetry.addData("mag", mag);
            m_telemetry.addData("cone?", r > 200);
            m_telemetry.addData("led enabled?", gamepad1.x);
            m_telemetry.update();
        }
    }

}
