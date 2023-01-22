package org.firstinspires.ftc.teamcode.BlakeStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "(Blake) Test Slides", group = "Blake")
public class BlakeTestSlides extends LinearOpMode {

    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;

    @Override
    public void runOpMode() {
        leftSlide = hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_slide");

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            leftSlide.setPower(gamepad2.left_stick_y);
            rightSlide.setPower(gamepad2.left_stick_y);

            telemetry.addLine("Slides:");
            telemetry.addLine();
            telemetry.addData("left pos", leftSlide.getCurrentPosition());
            telemetry.addData("right pos", rightSlide.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("left power", leftSlide.getPower());
            telemetry.addData("right power", rightSlide.getPower());
            telemetry.update();
        }
    }
}
