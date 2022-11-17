package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoBlueMid extends LinearOpMode {

    public DcMotor frontLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontRight = null;
    public DcMotor rearLeft = null;
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;
    public Servo Gripper = null;
    public DcMotor arm = null;

    private static final int Slides_Start = 0;
    private static final int Arm_Start = 0;
    private static final int Arm_Ground = -100;
    private static final int Slides_Low = -400;
    private static final int Arm_Low = 400;
    private static final int Slides_Medium = -750;
    private static final int Arm_Medium = 380;
    private static final int Slides_High = -1100;
    private static final int Arm_High = 350;
    private static final double Gripper_Release = 0.9;
    private static final double Gripper_Grab = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        // Define and Initialize Motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_wheel");
        rearLeft = hardwareMap.get(DcMotor.class, "rear_left_wheel");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_wheel");
        rearRight = hardwareMap.get(DcMotor.class, "rear_right_wheel");
        leftSlide = hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_slide");
        Gripper = hardwareMap.get(Servo.class, "left_intake");
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Reversing the motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);

        // Reset the slides
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setTargetPosition(Slides_Start);
        rightSlide.setTargetPosition(Slides_Start);
        arm.setTargetPosition(Arm_Start);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Gripper.setPosition(Gripper_Grab);

        // PID Values
        leftSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(5, 0, 0, 0));
        rightSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(5, 0, 0, 0));


        waitForStart();




    }


}
