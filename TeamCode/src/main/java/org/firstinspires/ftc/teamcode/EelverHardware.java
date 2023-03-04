package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1Constants;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2Constants;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.SlidesConstants;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.GripperConstants;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.IntakeServosConstants;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants;

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
    DistanceSensor distanceSensor = null;
    private int m_counter = 0;

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
        distanceSensor   = hardwareMap.get(DistanceSensor.class, "distance_sensor");

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

        leftSlide.  setTargetPosition(SlidesConstants.Start);
        rightSlide. setTargetPosition(SlidesConstants.Start);

        leftSlide.  setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide. setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Arm servos
        arm1.setPosition(Arm1Constants.Start);
        arm2.setPosition(Arm2Constants.Start);

        // Wiggle the arm servos so they (hopefully) don't
        // misbehave on the first two real commands
        arm1.setPosition(Arm1Constants.Start + 0.01);
        arm2.setPosition(Arm2Constants.Start + 0.01);
        arm1.setPosition(Arm1Constants.Start);
        arm2.setPosition(Arm2Constants.Start);

//        // PID Values
//        leftSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
//                new PIDFCoefficients(5, 0, 0, 0));
//        rightSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
//                new PIDFCoefficients(5, 0, 0, 0));

    }

    public boolean Distance() {
        return(distanceSensor.getDistance(DistanceUnit.MM) <= 30);
    }


    /*------- Gripper -------*/
    public boolean isGripping() {
        return gripper.getPosition() == GripperConstants.Gripper_Grab;
    }

    public void gripCone()
    {
        gripper.setPosition(MotorValuesConstants.GripperConstants.Gripper_Grab);
    }

    public void releaseCone()
    {
        gripper.setPosition(GripperConstants.Gripper_Release);
    }


    /*------- Arm -------*/
    public void incrementCount(){
        m_counter = (m_counter + 1) % 5;
    }

    public void decrementCount(){
        m_counter = (m_counter - 1) % 5;
    }

    public void armToPosition()
    {
        if (m_counter == 0)
            armToCone1();
        if (m_counter == 1)
            armToCone2();
        if (m_counter == 2)
            armToCone3();
        if (m_counter == 3)
            armToCone4();
        if (m_counter == 4)
            armToCone5();
    }

    public void armToStart()
    {
        arm1.setPosition(Arm1Constants.Start);
        arm2.setPosition(Arm2Constants.Start);
    }

    public void armToCone1()
    {
        arm1.setPosition(Arm1Constants.Cone1);
        arm2.setPosition(Arm2Constants.Cone1);
    }

    public void armToCone2()
    {
        arm1.setPosition(Arm1Constants.Cone2);
        arm2.setPosition(Arm2Constants.Cone2);
    }

    public void armToCone3()
    {
        arm1.setPosition(Arm1Constants.Cone3);
        arm2.setPosition(Arm2Constants.Cone3);
    }

    public void armToCone4()
    {
        arm1.setPosition(Arm1Constants.Cone4);
        arm2.setPosition(Arm2Constants.Cone4);
    }

    public void armToCone5()
    {
        arm1.setPosition(Arm1Constants.Cone5);
        arm2.setPosition(Arm2Constants.Cone5);
    }

    public void armToLow()
    {
        arm1.setPosition(Arm1Constants.Scoring);
        arm2.setPosition(Arm2Constants.Scoring);
    }

    public void armToMedium()
    {
        arm1.setPosition(Arm1Constants.Scoring);
        arm2.setPosition(Arm2Constants.Scoring);
    }

    public void armToHigh()
    {
        arm1.setPosition(Arm1Constants.Scoring);
        arm2.setPosition(Arm2Constants.Scoring);
    }

    public void armScoring()
    {
        arm1.setPosition(Arm1Constants.Scoring);
        arm2.setPosition(Arm2Constants.Scoring);
    }

    public void armWiggle()
    {
        armToStart();
        arm1.setPosition(Arm1Constants.Start + 0.01);
        arm2.setPosition(Arm2Constants.Start + 0.01);
        armToStart();
    }


    /*------- Slides -------*/
    public void setSlidesPower(double power)
    {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public void slidesToStart()
    {
        leftSlide.setTargetPosition(SlidesConstants.Start);
        rightSlide.setTargetPosition(SlidesConstants.Start);
    }

    public void slidesToGround()
    {
        leftSlide.setTargetPosition(SlidesConstants.Ground);
        rightSlide.setTargetPosition(SlidesConstants.Ground);
    }

    public void slidesToLow()
    {
        leftSlide.setTargetPosition(SlidesConstants.Low);
        rightSlide.setTargetPosition(SlidesConstants.Low);
    }

    public void slidesToMedium()
    {
        leftSlide.setTargetPosition(SlidesConstants.Medium);
        rightSlide.setTargetPosition(SlidesConstants.Medium);
    }

    public void slidesToHigh()
    {
        leftSlide.setTargetPosition(SlidesConstants.High);
        rightSlide.setTargetPosition(SlidesConstants.High);
    }

    public boolean slidesAreBusy()
    {
        return leftSlide.isBusy() || rightSlide.isBusy();
    }

    public void slidesDrop()
    {
        leftSlide.setTargetPosition(leftSlide.getTargetPosition()-200);
        rightSlide.setTargetPosition(rightSlide.getTargetPosition()-200);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public void slidesUp()
    {
        leftSlide.setTargetPosition(leftSlide.getTargetPosition()+200);
        rightSlide.setTargetPosition(rightSlide.getTargetPosition()+200);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public boolean MoveCone()
    {
        return leftSlide.isBusy() || rightSlide.isBusy();
    }


    /*------- Drivetrain -------*/
    public void setMecanumPower(double drive, double strafe, double twist, double slowMode)
    {
        frontLeft   .setPower((drive + strafe + twist) * slowMode);
        frontRight  .setPower((drive - strafe - twist) * slowMode);
        rearLeft    .setPower((drive - strafe + twist) * slowMode);
        rearRight   .setPower((drive + strafe - twist) * slowMode);
    }


    /*------- Intake -------*/
    public void intake(double power) {
        Intake.setPower(power);
    }

    public void intakeIn() {
        Intake.setPower(1);
    }

    public void intakeOut() {
        Intake.setPower(-0.5);
    }

    public void intakeServoIn() {
        intakeLeft.setPosition(IntakeServosConstants.IntakeLeft_in);
        intakeRight.setPosition(IntakeServosConstants.IntakeRight_in);
    }

    public void intakeServoOut() {
        intakeLeft.setPosition(IntakeServosConstants.IntakeLeft_out);
        intakeRight.setPosition(IntakeServosConstants.IntakeRight_out);
    }
}
