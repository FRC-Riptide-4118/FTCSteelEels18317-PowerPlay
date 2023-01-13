package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "TeleOp", group = "Robot")

public class TeleOpTesting extends LinearOpMode {


    /* Declare OpMode members. */
    public Servo Gripper = null;
    public Servo arm1 = null;
    public Servo arm2 = null;
    // public DcMotor Intake = null;

    //Arm Encoder Values
    private static final double Arm_Start = 0;
    private static final double Arm_Ground = .2;
    private static final double Arm_Low = .2;
    private static final double Arm_Medium = 0;
    private static final double Arm_High = .1;

    //Gripper Values
    private static final double Gripper_Grab = 0;
    private static final double Gripper_Release = 0.4;

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
        // Intake = hardwareMap.get(DcMotor.class, "Intake");

        arm2.setPosition(Arm_Start);
        arm1.setPosition(Arm_Start);
        Gripper.setPosition(Gripper_Grab);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*-------Arm-------*/
            // Ground
            if(gamepad1.a) {
                arm1.setPosition(Arm_Ground);
            }

            // Low
            if(gamepad1.x) {
                arm1.setPosition(Arm_Low);
            }


            // Medium
            if(gamepad1.y) {
                arm1.setPosition(Arm_Medium);
            }


            // High
            if(gamepad1.b) {
                arm1.setPosition(Arm_High);
            }



            if (gamepad1.dpad_up) {
                Gripper.setPosition(Gripper_Grab);
            }

            if (gamepad1.dpad_down) {
                Gripper.setPosition(Gripper_Release);
            }




            /*-------Intake-------*/
            // Intake.setPower(-gamepad1.left_trigger);
            // Intake.setPower(gamepad1.right_trigger);

        }
    }
}
