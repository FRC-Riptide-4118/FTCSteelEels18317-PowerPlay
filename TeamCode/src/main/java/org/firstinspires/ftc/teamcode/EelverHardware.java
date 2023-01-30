package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_High;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Start;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_High;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_Start;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Grab;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_High;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_Start;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class EelverHardware {

    HardwareMap hardwareMap;

    // Hardware components (motors, sensors, servos, etc)
    public DcMotor frontLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontRight = null;
    public DcMotor rearLeft = null;
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;
    public Servo Gripper = null;
    public Servo arm1 = null;
    public Servo arm2 = null;


    public void init(HardwareMap hwMap)
    {
        /*------- Initialize hardware -------*/
        frontLeft =     hardwareMap.get(DcMotor.class, "front_left_wheel");
        rearLeft =      hardwareMap.get(DcMotor.class, "rear_left_wheel");
        frontRight =    hardwareMap.get(DcMotor.class, "front_right_wheel");
        rearRight =     hardwareMap.get(DcMotor.class, "rear_right_wheel");
        leftSlide =     hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlide =    hardwareMap.get(DcMotorEx.class, "right_slide");
        Gripper =       hardwareMap.get(Servo.class, "Gripper");
        arm1 =          hardwareMap.get(Servo.class, "arm1");
        arm2 =          hardwareMap.get(Servo.class, "arm2");

        /*------- Do hardware setup -------*/

        // Drive motors
        frontLeft.  setDirection(DcMotor.Direction.REVERSE);
        rearLeft.   setDirection(DcMotor.Direction.REVERSE);
        frontRight. setDirection(DcMotor.Direction.FORWARD);
        rearRight.  setDirection(DcMotor.Direction.FORWARD);

        // Slide motors
        leftSlide.  setDirection(DcMotor.Direction.REVERSE);
        rightSlide. setDirection(DcMotor.Direction.REVERSE);

        leftSlide.  setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide. setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.  setTargetPosition(Slides_Start);
        rightSlide. setTargetPosition(Slides_Start);

        leftSlide.  setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide. setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Arm servos
        arm1.setPosition(Arm1_Start);
        arm2.setPosition(Arm2_Start);

        // Wiggle the arm servos so they (hopefully) don't
        // misbehave on the first two real commands
        arm1.setPosition(Arm1_Start + 0.01);
        arm2.setPosition(Arm2_Start + 0.01);
        arm1.setPosition(Arm1_Start);
        arm2.setPosition(Arm2_Start);
    }


    public void gripCone()
    {
        Gripper.setPosition(Gripper_Grab);
    }

    public void releaseCone()
    {
        Gripper.setPosition(Gripper_Grab);
    }

    public void armToStart()
    {
        arm1.setPosition(Arm1_Start);
        arm2.setPosition(Arm2_Start);
    }

    public void armToHigh()
    {
        arm1.setPosition(Arm1_High);
        arm2.setPosition(Arm2_High);
    }

    public void armWiggle()
    {
        armToStart();
        arm1.setPosition(Arm1_Start + 0.01);
        arm2.setPosition(Arm2_Start + 0.01);
        armToStart();
    }

    public void setSlidesPower(double power)
    {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }


    public void slidesToStart()
    {
        leftSlide.setTargetPosition(Slides_Start);
        rightSlide.setTargetPosition(Slides_Start);
    }

    public void slidesToHigh()
    {
        leftSlide.setTargetPosition(Slides_High);
        rightSlide.setTargetPosition(Slides_High);
    }

    public boolean slidesAreBusy()
    {
        return leftSlide.isBusy() || rightSlide.isBusy();
    }
}
