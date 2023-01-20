package org.firstinspires.ftc.teamcode.Auto.OldAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.apriltag.AprilTagDetection;

@Autonomous(name = "WheelsTesting")

public class WheelsTesting extends LinearOpMode {

    private ColorSensor colorSensor = null;
    private DcMotor  frontLeft  = null;
    private DcMotor  rearRight  = null;
    private DcMotor  frontRight  = null;
    private DcMotor  rearLeft  = null;
    private DcMotorEx  leftSlide  = null;
    private DcMotorEx  rightSlide  = null;
    private Servo  Gripper = null;
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

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int HD_COUNTS_PER_REV;
        int DRIVE_GEAR_REDUCTION;
        double WHEEL_CIRCUMFERENCE_MM;
        double DRIVE_COUNTS_PER_MM;

        frontLeft = hardwareMap.get(DcMotor.class, "front_left_wheel");
        rearLeft = hardwareMap.get(DcMotor.class, "rear_left_wheel");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_wheel");
        rearRight = hardwareMap.get(DcMotor.class, "rear_right_wheel");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        // Math for Traveling w/ Inches
        while (opModeIsActive()) {
            // Running Code
            frontLeft.setPower(.5);
            rearLeft.setPower(.5);
            frontRight.setPower(.5);
            rearRight.setPower(.5);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("frontLeft_currentPosition", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight_currentPosition", frontRight.getCurrentPosition());
            telemetry.addData("rearLeft_currentPosition", rearLeft.getCurrentPosition());
            telemetry.addData("rearRight_currentPosition", rearRight.getCurrentPosition());
            telemetry.update();
        }

    }
}

