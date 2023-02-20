package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Grab;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Release;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.SlidesConstants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOp", group = "Robot")

@Config
public class TeleOpTesting extends LinearOpMode {

    /* Declare OpMode members. */
    public Servo Gripper = null;
    public Servo arm1 = null;
    public Servo arm2 = null;
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;
    public  DistanceSensor distanceSensor = null;

    // Toggling
    boolean pressedLastIteration = false;

    @Override
    public void runOpMode() {
        // Define and Initialize Motors
        Gripper          = hardwareMap.get(Servo.class, "Gripper");
        arm1             = hardwareMap.get(Servo.class, "arm1");
        arm2             = hardwareMap.get(Servo.class, "arm2");
        leftSlide        = hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlide       = hardwareMap.get(DcMotorEx.class, "right_slide");
        distanceSensor   = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        // Intake = hardwareMap.get(DcMotor.class, "Intake");

        leftSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(5, 0, 0, 0));
        rightSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(5, 0, 0, 0));

        arm1.setPosition(Arm1.Start);
        arm2.setPosition(Arm2.Start);
//        Gripper.setPosition(Gripper_Grab);

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        // Reset the slides
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Slides
        leftSlide.setTargetPosition(SlidesConstants.Start);
        rightSlide.setTargetPosition(SlidesConstants.Start);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*-------Arm-------*/
            // Ground
            if(gamepad1.a) {
                arm1.setPosition(Arm1.Cone1);
                arm2.setPosition(Arm2.Cone1);
            }

            // Low
            if(gamepad1.x) {
                arm1.setPosition(Arm1.Front);
                arm2.setPosition(Arm2.Front);
            }

            // Medium
            if(gamepad1.y) {
                arm1.setPosition(Arm1.Scoring);
                arm2.setPosition(Arm2.Scoring);
            }

            // High
            if(gamepad1.b) {
                arm1.setPosition(Arm1.Medium);
                arm2.setPosition(Arm2.Medium);
            }

            /*-------Slides-------*/
            // Grounds
            if(gamepad2.a) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                leftSlide.setTargetPosition(SlidesConstants.Start);
                rightSlide.setTargetPosition(SlidesConstants.Start);
            }

            // Low
            if(gamepad2.x) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                leftSlide.setTargetPosition(SlidesConstants.Low);
                rightSlide.setTargetPosition(SlidesConstants.Low);
            }

            // Medium
            if(gamepad2.y) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                leftSlide.setTargetPosition(SlidesConstants.Medium);
                rightSlide.setTargetPosition(SlidesConstants.Medium);
            }

            // High
            if(gamepad2.b) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                leftSlide.setTargetPosition(SlidesConstants.High);
                rightSlide.setTargetPosition(SlidesConstants.High);
            }

            if(gamepad2.right_bumper) {
                arm1.setPosition(Arm1.Ground);
                arm2.setPosition(Arm2.Ground);
            }

            if (gamepad1.dpad_up) {
                Gripper.setPosition(Gripper_Grab);
            }

            if (gamepad1.dpad_down) {
                Gripper.setPosition(Gripper_Release);
            }
                telemetry.addData("Arm1 Pos", arm1.getPosition());
                telemetry.addData("Arm2 Pos", arm2.getPosition());
                telemetry.addData("Gripper Distance", distanceSensor.getDistance(DistanceUnit.CM));
                telemetry.addLine();
                telemetry.addData("left pos", leftSlide.getCurrentPosition());
                telemetry.addData("right pos", rightSlide.getCurrentPosition());
                telemetry.addLine();
                telemetry.addData("left power", leftSlide.getPower());
                telemetry.addData("right power", rightSlide.getPower());
                telemetry.addLine();
                telemetry.addData("left target", leftSlide.getTargetPosition());
                telemetry.addData("right target", rightSlide.getTargetPosition());
                telemetry.update();


            // Fine Control the Slides
            if(gamepad2.dpad_up) {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + 50);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 50);
            }
            if(gamepad2.dpad_down) {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() - 50);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - 50);
            }

        }
    }
}
