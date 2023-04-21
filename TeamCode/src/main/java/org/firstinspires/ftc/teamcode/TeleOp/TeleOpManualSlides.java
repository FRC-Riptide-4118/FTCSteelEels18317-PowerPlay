package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@Config
@TeleOp(group = "Worlds")
public class TeleOpManualSlides extends LinearOpMode {

    public static int delta = 50;

    public void runOpMode() {
        Slides slides = new Slides(hardwareMap);


        boolean dpuPLI = false;
        boolean dpdPLI = false;

        waitForStart();

        while(opModeIsActive()) {

            if(gamepad1.dpad_up && !dpuPLI) {
                slides.setPosition(slides.getLeftPosition() + delta);
            } else if(gamepad1.dpad_down && !dpdPLI) {
                slides.setPosition(slides.getLeftPosition() - delta);
            }

            dpuPLI = gamepad1.dpad_up;
            dpdPLI = gamepad1.dpad_down;

            if(gamepad1.y) slides.resetEncoders();

            telemetry.addData("slides position", slides.getLeftPosition());
            telemetry.update();
        }
    }
}
