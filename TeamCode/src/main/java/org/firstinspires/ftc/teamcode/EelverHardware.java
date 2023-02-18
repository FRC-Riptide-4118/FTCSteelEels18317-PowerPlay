package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm1_Scoring;
import static org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm2_Scoring;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Grab;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Release;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.IntakeServos;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public Servo intakeLeft = null;
    public Servo intakeRight = null;

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
        intakeLeft =       hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight =       hardwareMap.get(Servo.class, "intakeRight");

        /*------- Do hardware setup -------*/

        // Drive motors
        frontLeft.  setDirection(DcMotor.Direction.REVERSE);
        rearLeft.   setDirection(DcMotor.Direction.REVERSE);
        frontRight. setDirection(DcMotor.Direction.FORWARD);
        rearRight.  setDirection(DcMotor.Direction.FORWARD);

        // Slide motors
        leftSlide.  setDirection(DcMotor.Direction.REVERSE);
        rightSlide. setDirection(DcMotor.Direction.FORWARD);

        leftSlide.  setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide. setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.  setTargetPosition(Slides.Start);
        rightSlide. setTargetPosition(Slides.Start);

        leftSlide.  setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide. setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Arm servos
        arm1.setPosition(Arm1.Start);
        arm2.setPosition(Arm2.Start);

        // Wiggle the arm servos so they (hopefully) don't
        // misbehave on the first two real commands
        arm1.setPosition(Arm1.Start + 0.01);
        arm2.setPosition(Arm2.Start + 0.01);
        arm1.setPosition(Arm1.Start);
        arm2.setPosition(Arm2.Start);

        // PID Values
        leftSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(5, 0, 0, 0));
        rightSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(5, 0, 0, 0));

        // Intake
        Intake.     setDirection(DcMotor.Direction.REVERSE);
        intakeServoOut();
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
        arm1.setPosition(Arm1.Start);
        arm2.setPosition(Arm2.Start);
    }

    public void armToCone1()
    {
        arm1.setPosition(Arm1.Cone1);
        arm2.setPosition(Arm2.Cone1);
    }

    public void armToLow()
    {
        arm1.setPosition(Arm1.Low);
        arm2.setPosition(Arm2.Low);
    }

    public void armToMedium()
    {
        arm1.setPosition(Arm1.Medium);
        arm2.setPosition(Arm2.Medium);
    }

    public void armToHigh()
    {
        arm1.setPosition(Arm1.High);
        arm2.setPosition(Arm2.High);
    }

    public void armScoring()
    {
        arm1.setPosition(Arm1_Scoring);
        arm2.setPosition(Arm2_Scoring);
    }

    public void armWiggle()
    {
        armToStart();
        arm1.setPosition(Arm1.Start + 0.01);
        arm2.setPosition(Arm2.Start + 0.01);
        armToStart();
    }

    public void setSlidesPower(double power)
    {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public void slidesToStart()
    {
        leftSlide.setTargetPosition(Slides.Ground);
        rightSlide.setTargetPosition(Slides.Ground);
    }

    public void slidesToLow()
    {
        leftSlide.setTargetPosition(Slides.Low);
        rightSlide.setTargetPosition(Slides.Low);
    }

    public void slidesToMedium()
    {
        leftSlide.setTargetPosition(Slides.Medium);
        rightSlide.setTargetPosition(Slides.Medium);
    }

    public void slidesToHigh()
    {
        leftSlide.setTargetPosition(Slides.High);
        rightSlide.setTargetPosition(Slides.High);
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

    public void intake(double power) {
        Intake.setPower(power);
    }

    public void intakeIn() {
        Intake.setPower(.5);
    }

    public void intakeOut() {
        Intake.setPower(-.5);
    }

    public void intakeServoIn() {
        intakeLeft.setPosition(IntakeServos.IntakeLeft_in);
        intakeRight.setPosition(IntakeServos.IntakeRight_in);
    }

    public void intakeServoOut() {
        intakeLeft.setPosition(IntakeServos.IntakeLeft_out);
        intakeRight.setPosition(IntakeServos.IntakeRight_out);
    }
}
