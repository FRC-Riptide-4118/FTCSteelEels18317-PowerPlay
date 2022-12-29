package org.firstinspires.ftc.teamcode.Auto.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TestingSlidesOnly")

public class TestingSlidesOnly extends LinearOpMode {

    private DcMotorEx leftSlide = null;
    private DcMotorEx rightSlide = null;
    private Servo Gripper = null;
    private DcMotor arm = null;

    //Slides Encoder Values
    private static final int Slides_Start = 0;
    private static final int Slides_Low = -400;
    private static final int Slides_Medium = -900;
    private static final int Slides_High = -1100;

    //Arm Encoder Values
    private static final int Arm_Start = 0;
    private static final int Arm_Ground = -100;
    private static final int Arm_Low = 420;
    private static final int Arm_Medium = 420;
    private static final int Arm_High = 350;

    //Gripper Values
    private static final double Gripper_Release = 0.7;
    private static final double Gripper_Grab = 0;

    private boolean raisingToLow = false;
    private boolean returning = false;
    private boolean raisingToMiddle = false;
    private ElapsedTime armInTimer;

    int frontRightTarget;
    int frontLeftTarget;
    int rearRightTarget;
    int rearLeftTarget;
    double DRIVE_COUNTS_PER_IN;

    @Override
    public void runOpMode() {
        Gripper = hardwareMap.get(Servo.class, "left_intake");
        arm = hardwareMap.get(DcMotor.class, "arm");
        leftSlide = hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_slide");

        Gripper.setPosition(Gripper_Grab);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setTargetPosition(Slides_Start);
        rightSlide.setTargetPosition(Slides_Start);
        arm.setTargetPosition(Arm_Start);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // PID Values
        leftSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(5, 0, 0, 0));
        rightSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(5, 0, 0, 0));

        waitForStart();

        armInTimer = new ElapsedTime();
        armInTimer.reset();

        raiseCone();
        telemetry.addLine("done raising cone");
        telemetry.update();
        sleep(10000);
        lowerCone();
        telemetry.addLine("done lowering cone");
        telemetry.update();

        while (opModeIsActive()) {

            telemetry.addData("LeftSlidePos", leftSlide.getCurrentPosition());
            telemetry.addData("RightSlidePos", rightSlide.getCurrentPosition());
            telemetry.addData("ArmCurrentPos", arm.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("LeftSlideTarPos", leftSlide.getTargetPosition());
            telemetry.addData("RightSlideTarPos", rightSlide.getTargetPosition());
            telemetry.addData("ArmCurrentTarPos", arm.getTargetPosition());

        }
    }
    // Raising the Cone
    public void raiseCone () {
        Gripper.setPosition(Gripper_Grab);

        leftSlide.setPower(1);
        rightSlide.setPower(1);
        leftSlide.setTargetPosition(Slides_Medium);
        rightSlide.setTargetPosition(Slides_Medium);

        // Raising
        while (opModeIsActive() && leftSlide.getCurrentPosition() < -750);
        arm.setPower(.5);
        arm.setTargetPosition(Arm_Medium);
    }


    // Lowering the Cone
    public void lowerCone () {
        Gripper.setPosition(Gripper_Grab);

        arm.setPower(.5);
        arm.setTargetPosition(Arm_Ground);
        while(opModeIsActive() && arm.isBusy());

        // Returning
        leftSlide.setPower(0.8);
        rightSlide.setPower(0.8);
        leftSlide.setTargetPosition(Slides_Start);
        rightSlide.setTargetPosition(Slides_Start);
        Gripper.setPosition(Gripper_Release);
        arm.setTargetPosition(Arm_Start);
    }
}