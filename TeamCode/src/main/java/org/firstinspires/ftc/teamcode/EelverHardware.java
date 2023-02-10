package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Cone1;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_High;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Low;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Medium;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Start;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_Cone1;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_High;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_Low;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_Medium;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_Start;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Grab;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Release;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_Ground;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_High;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_Low;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_Medium;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_Start;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Intake1_in;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Intake1_out;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Intake2_in;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Intake2_out;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class EelverHardware {

    // Hardware components (motors, sensors, servos, etc)
    public DcMotor frontLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontRight = null;
    public DcMotor rearLeft = null;
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;
    public Servo gripper = null;
    public Servo arm1 = null;
    public Servo arm2 = null;
    public Servo conePush = null;
    public DcMotor Intake = null;
    public Servo Intake1 = null;
    public Servo Intake2 = null;

    //    OpenCV class
    public void init(HardwareMap hardwareMap)
    {
        /*------- Initialize hardware -------*/
        frontLeft =     hardwareMap.get(DcMotor.class, "front_left_wheel");
        rearLeft =      hardwareMap.get(DcMotor.class, "rear_left_wheel");
        frontRight =    hardwareMap.get(DcMotor.class, "front_right_wheel");
        rearRight =     hardwareMap.get(DcMotor.class, "rear_right_wheel");
        leftSlide =     hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlide =    hardwareMap.get(DcMotorEx.class, "right_slide");
        gripper =       hardwareMap.get(Servo.class, "Gripper");
        arm1 =          hardwareMap.get(Servo.class, "arm1");
        arm2 =          hardwareMap.get(Servo.class, "arm2");
        // conePush =      hardwareMap.get(Servo.class, "cone");
        Intake =        hardwareMap.get(DcMotor.class, "Intake");
        Intake1 =       hardwareMap.get(Servo.class, "Intake1");
        Intake2 =       hardwareMap.get(Servo.class, "Intake2");

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

        // PID Values
        leftSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(5, 0, 0, 0));
        rightSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(5, 0, 0, 0));

        // Intake
        Intake.     setDirection(DcMotor.Direction.REVERSE);
    }

    public void gripCone()
    {
        gripper.setPosition(Gripper_Grab);
    }

    public void releaseCone()
    {
        gripper.setPosition(Gripper_Release);
    }

    public void armToStart()
    {
        arm1.setPosition(Arm1_Start);
        arm2.setPosition(Arm2_Start);
    }

    public void armToCone1()
    {
        arm1.setPosition(Arm1_Cone1);
        arm2.setPosition(Arm2_Cone1);
    }

    public void armToLow()
    {
        arm1.setPosition(Arm1_Low);
        arm2.setPosition(Arm2_Low);
    }

    public void armToMedium()
    {
        arm1.setPosition(Arm1_Medium);
        arm2.setPosition(Arm2_Medium);
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
        leftSlide.setTargetPosition(Slides_Ground);
        rightSlide.setTargetPosition(Slides_Ground);
    }

    public void slidesToLow()
    {
        leftSlide.setTargetPosition(Slides_Low);
        rightSlide.setTargetPosition(Slides_Low);
    }

    public void slidesToMedium()
    {
        leftSlide.setTargetPosition(Slides_Medium);
        rightSlide.setTargetPosition(Slides_Medium);
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

    public boolean MoveCone()
    {
        return leftSlide.isBusy() || rightSlide.isBusy();
    }

    public void setMecanumPower(double drive, double strafe, double twist, double slowMode)
    {
        frontLeft   .setPower((drive + strafe + twist) * slowMode);
        frontRight  .setPower((drive - strafe - twist) * slowMode);
        rearLeft    .setPower((drive - strafe + twist) * slowMode);
        rearRight   .setPower((drive + strafe - twist) * slowMode);
    }

    public void intakeIn() {
        Intake.setPower(.5);
    }

    public void intakeOut() {
        Intake.setPower(-.5);
    }

    public void intakeServoIn() {
        Intake1.setPosition(Intake1_in);
        Intake2.setPosition(Intake2_in);
    }

    public void intakeServoOut() {
        Intake1.setPosition(Intake1_out);
        Intake2.setPosition(Intake2_out);
    }
}
