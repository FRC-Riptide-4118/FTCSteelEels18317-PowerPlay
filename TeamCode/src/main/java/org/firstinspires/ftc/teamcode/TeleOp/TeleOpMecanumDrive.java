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
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.TeleOpSlides_Ground;
import org.firstinspires.ftc.teamcode.EelverHardware;

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

    int leftSlidePrevPos;

    // Toggling
    boolean pressedLastIteration = false;

    @Override
    public void runOpMode() {
        EelverHardware hardware = new EelverHardware();
        hardware.init(hardwareMap);

        armInTimer = new ElapsedTime();
        armInTimer.reset();

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
            double slowMode = Math.abs(1.25 - gamepad1.left_trigger);

            hardware.setMecanumPower(drive, strafe, twist, slowMode);

            /*-------Gripper-------*/
            boolean pressed = gamepad1.left_bumper;
            if (pressed & !pressedLastIteration) {

                if(Gripper.getPosition() == Gripper_Grab) {
                    hardware.releaseCone();
                }
                else {
                    hardware.gripCone();
                }
            }
            pressedLastIteration = pressed;

            /*-------Lift & Arm-------*/
            // Ground
            if(gamepad1.a) {
                returning = true;
                hardware.gripCone();
                hardware.armToStart();
                if (armInTimer.seconds() > 1.0) armInTimer.reset();
            }

            if(returning) {
                if(armInTimer.seconds() > 1.0) {
                    hardware.setSlidesPower(0.7);
                    hardware.slidesToStart();
                    hardware.releaseCone();
                    hardware.armToStart();
                    hardware.armToStart();
                    returning = false;
                }
            }

            // Low
            if(gamepad1.x) {
                raisingToLow = true;
                hardware.gripCone();
                hardware.setSlidesPower(1);
                hardware.slidesToLow();
            }
            if(raisingToLow) {
                if(leftSlide.getCurrentPosition() > 600) {
                    hardware.armToLow();
                    raisingToLow = false;
                }
            }

            // Medium
            if(gamepad1.y) {
                raisingToMiddle = true;
                hardware.gripCone();
                hardware.setSlidesPower(1);
                hardware.slidesToMedium();
            }
            if(raisingToMiddle) {
                if(leftSlide.getCurrentPosition() > 600) {
                    hardware.armToMedium();
                    raisingToMiddle = false;
                }
            }

            // High
            if(gamepad1.b) {
                raisingToHigh = true;
                hardware.gripCone();
                hardware.setSlidesPower(1);
                hardware.slidesToHigh();
            }
            if(raisingToHigh) {
                if(leftSlide.getCurrentPosition() > 600) {
                    hardware.armToHigh();
                    raisingToHigh = false;
                }
            }

            // Fine Control the Slides
            if(gamepad1.dpad_down) {
                leftSlidePrevPos = leftSlide.getTargetPosition();
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + 50);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 50);
            }
            if(gamepad1.dpad_up) {
                leftSlidePrevPos = leftSlide.getTargetPosition();
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() - 50);
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - 50);
            }

            /*-------Intake-------*/
            telemetry.addData("left pos", leftSlide.getCurrentPosition());
            telemetry.addData("right pos", rightSlide.getCurrentPosition());
            telemetry.addLine();
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

