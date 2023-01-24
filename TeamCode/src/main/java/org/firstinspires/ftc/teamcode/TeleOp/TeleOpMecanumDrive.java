/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Ground;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_High;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Low;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Medium;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Start;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_Ground;
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

import org.firstinspires.ftc.teamcode.BlakeStuff.BlakeRobotHardware;

@TeleOp(name = "TeleOpMecanumDrive", group = "Robot")

public class TeleOpMecanumDrive extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor frontLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontRight = null;
    public DcMotor rearLeft = null;
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;
    public Servo Gripper = null;
    public Servo arm1 = null;
    public Servo arm2 = null;
    // public DcMotor Intake = null;

    private boolean raisingToLow = false;
    private boolean returning = false;
    private boolean raisingToMiddle = false;
    private boolean raisingToHigh = false;
    private ElapsedTime armInTimer;

    // Toggling
    boolean pressedLastIteration = false;
    BlakeRobotHardware hardware;

    @Override
    public void runOpMode() {
        // Define and Initialize Motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_wheel");
        rearLeft = hardwareMap.get(DcMotor.class, "rear_left_wheel");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_wheel");
        rearRight = hardwareMap.get(DcMotor.class, "rear_right_wheel");
        leftSlide = hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_slide");
        Gripper = hardwareMap.get(Servo.class, "Gripper");
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        // Intake = hardwareMap.get(DcMotor.class, "Intake");

        // Reversing the motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        // Reset the slides

        arm1.setPosition(Arm1_Start);
        arm2.setPosition(Arm2_Start);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setTargetPosition(Slides_Start);
        rightSlide.setTargetPosition(Slides_Start);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // PID Values
//        leftSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
//                new PIDFCoefficients(5, 0, 0, 0));
//        rightSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
//                new PIDFCoefficients(5, 0, 0, 0));

        armInTimer = new ElapsedTime();
        armInTimer.reset();

        hardware = new BlakeRobotHardware(hardwareMap);
        hardware.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver1");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*-------Drivetrain-------*/
            // Gamepad controls
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;
            double SlowMode = 1.0 - gamepad1.left_trigger;

            hardware.setMecanumPower(drive, strafe, twist);


            /*-------Gripper-------*/
            boolean pressed = gamepad1.left_bumper;
            if (pressed & !pressedLastIteration) {

                if(Gripper.getPosition() == Gripper_Grab) {
                    Gripper.setPosition(Gripper_Release);
                }
                else {
                    Gripper.setPosition(Gripper_Grab);
                }
            }
            pressedLastIteration = pressed;

            /*-------Lift & Arm-------*/

            // Ground
            if(gamepad1.a) {
                returning = true;
                Gripper.setPosition(Gripper_Grab);
                arm1.setPosition(Arm1_Ground);
                arm2.setPosition(Arm2_Ground);
                if (armInTimer.seconds() > 1.0) armInTimer.reset();
            }

            if(returning) {
                if(armInTimer.seconds() > 1.0) {
                    leftSlide.setPower(0.5);
                    rightSlide.setPower(0.5);
                    leftSlide.setTargetPosition(Slides_Ground);
                    rightSlide.setTargetPosition(Slides_Ground);
                    Gripper.setPosition(Gripper_Release);
                    arm1.setPosition(Arm1_Start);
                    arm2.setPosition(Arm2_Start);
                    returning = false;
                }
            }

            // Low
            if(gamepad1.x) {
                raisingToLow = true;
                Gripper.setPosition(Gripper_Grab);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                leftSlide.setTargetPosition(Slides_Low);
                rightSlide.setTargetPosition(Slides_Low);
            }
            if(raisingToLow) {
                if(leftSlide.getCurrentPosition() > 600) {
                    arm1.setPosition(Arm1_Low);
                    arm2.setPosition(Arm2_Low);
                    raisingToLow = false;
                }
            }

            // Medium
            if(gamepad1.y) {
                raisingToMiddle = true;
                Gripper.setPosition(Gripper_Grab);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                leftSlide.setTargetPosition(Slides_Medium);
                rightSlide.setTargetPosition(Slides_Medium);
            }
            if(raisingToMiddle) {
                if(leftSlide.getCurrentPosition() > 1100) {
                    arm1.setPosition(Arm1_Medium);
                    arm2.setPosition(Arm2_Medium);
                    raisingToMiddle = false;
                }
            }

            // High
            if(gamepad1.b) {
                raisingToHigh = true;
                Gripper.setPosition(Gripper_Grab);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                leftSlide.setTargetPosition(Slides_High);
                rightSlide.setTargetPosition(Slides_High);
            }
            if(raisingToHigh) {
                if(leftSlide.getCurrentPosition() > 1800) {
                    arm1.setPosition(Arm1_High);
                    arm2.setPosition(Arm2_High);
                    raisingToHigh = false;
                }
            }

            // Fine Control the Slides
            if(gamepad1.dpad_down) {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + 50);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 50);
            }
            if(gamepad1.dpad_up) {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() - 50);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - 50);
            }

            /*-------Intake-------*/
            // Intake.setPower(-gamepad1.left_trigger);
            // Intake.setPower(gamepad1.right_trigger);

            telemetry.addData("front left power", frontLeft.getPower());
            telemetry.addData("front right power", frontRight.getPower());
            telemetry.addData("back left power", rearLeft.getPower());
            telemetry.addData("back right power", rearRight.getPower());
            telemetry.addLine();
            telemetry.addData("front left pos", frontLeft.getCurrentPosition());
            telemetry.addData("front right pos", frontRight.getCurrentPosition());
            telemetry.addData("back left pos", rearLeft.getCurrentPosition());
            telemetry.addData("back right pos", rearRight.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("front left tarpos", frontLeft.getTargetPosition());
            telemetry.addData("front right tarpos", frontRight.getTargetPosition());
            telemetry.addData("back left tarpos", rearLeft.getTargetPosition());
            telemetry.addData("back right tarpos", rearRight.getTargetPosition());
            telemetry.addLine();
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Right Stick X", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}

