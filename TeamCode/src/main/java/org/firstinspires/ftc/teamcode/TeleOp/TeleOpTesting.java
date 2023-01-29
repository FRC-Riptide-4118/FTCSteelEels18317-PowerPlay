package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Front;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Ground;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_High;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Low;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Medium;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Scoring;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Start;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_Front;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_Ground;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_High;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_Low;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_Medium;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_Scoring;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_Start;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Grab;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Release;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_High;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_Low;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_Medium;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_Start;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp", group = "Robot")

@Config
public class TeleOpTesting extends LinearOpMode {

    /* Declare OpMode members. */
    public Servo Gripper = null;
    public Servo arm1 = null;
    public Servo arm2 = null;
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;
    // public DcMotor Intake = null;

    private boolean raisingToLow = false;
    private boolean returning = false;
    private boolean raisingToMiddle = false;
    private boolean raisingToHigh = false;
    private ElapsedTime armInTimer;

    // Toggling
    boolean pressedLastIteration = false;

    @Override
    public void runOpMode() {
        // Define and Initialize Motors
        Gripper = hardwareMap.get(Servo.class, "Gripper");
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        leftSlide = hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_slide");
        // Intake = hardwareMap.get(DcMotor.class, "Intake");

        leftSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(5, 0, 0, 0));
        rightSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(5, 0, 0, 0));

        arm1.setPosition(Arm1_Start);
        arm2.setPosition(Arm2_Start);
//        Gripper.setPosition(Gripper_Grab);

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        // Reset the slides
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Slides
        leftSlide.setTargetPosition(Slides_Start);
        rightSlide.setTargetPosition(Slides_Start);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*-------Arm-------*/
            // Ground
            if(gamepad1.a) {
                arm1.setPosition(Arm1_Ground);
                arm2.setPosition(Arm2_Ground);
//                leftSlide.setPower(.7);
//                rightSlide.setPower(.7);
//                leftSlide.setTargetPosition(Slides_Start);
//                rightSlide.setTargetPosition(Slides_Start);
            }

            // Low
            if(gamepad1.x) {
                arm1.setPosition(Arm1_Front);
                arm2.setPosition(Arm2_Front);
//                leftSlide.setPower(.7);
//                rightSlide.setPower(.7);
//                leftSlide.setTargetPosition(Slides_Low);
//                rightSlide.setTargetPosition(Slides_Low);
            }

            // Medium
            if(gamepad1.y) {
                arm1.setPosition(Arm1_Scoring);
                arm2.setPosition(Arm2_Scoring);
//                leftSlide.setPower(.7);
//                rightSlide.setPower(.7);
//                leftSlide.setTargetPosition(Slides_Medium);
//                rightSlide.setTargetPosition(Slides_Medium);
            }

            // High
            if(gamepad1.b) {
                arm1.setPosition(Arm1_Medium);
                arm2.setPosition(Arm2_Medium);
//                leftSlide.setPower(.7);
//                rightSlide.setPower(.7);
//                leftSlide.setTargetPosition(Slides_High);
//                rightSlide.setTargetPosition(Slides_High);
            }

            /*-------Slides-------*/
            // Ground
            if(gamepad2.a) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                leftSlide.setTargetPosition(Slides_Start);
                rightSlide.setTargetPosition(Slides_Start);
            }

            // Low
            if(gamepad2.x) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                leftSlide.setTargetPosition(Slides_Low);
                rightSlide.setTargetPosition(Slides_Low);
            }

            // Medium
            if(gamepad2.y) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                leftSlide.setTargetPosition(Slides_Medium);
                rightSlide.setTargetPosition(Slides_Medium);
            }

            // High
            if(gamepad2.b) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                leftSlide.setTargetPosition(Slides_High);
                rightSlide.setTargetPosition(Slides_High);
            }

            if(gamepad2.right_bumper) {
                arm1.setPosition(Arm1_Ground);
                arm2.setPosition(Arm2_Ground);
            }

            if (gamepad1.dpad_up) {
                Gripper.setPosition(Gripper_Grab);
            }

            if (gamepad1.dpad_down) {
                Gripper.setPosition(Gripper_Release);
            }


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

            /*-------Intake-------*/
            // Intake.setPower(-gamepad1.left_trigger);
            // Intake.setPower(gamepad1.right_trigger);

        }
    }
}
