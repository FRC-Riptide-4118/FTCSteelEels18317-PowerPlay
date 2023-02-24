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
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.GripperConstants;

import org.firstinspires.ftc.teamcode.EelverHardware;

@TeleOp(name = "TeleOpMecanumDrive", group = "Robot")

public class TeleOpMecanumDrive extends LinearOpMode {

    private boolean raisingToLow    = false;
    private boolean returning       = false;
    private boolean raisingToMiddle = false;
    private boolean raisingToHigh   = false;
    private boolean atHigh          = false;
    private boolean atMid           = false;
    private boolean atLow           = false;

    private ElapsedTime armInTimer;

    private boolean pressedLastIteration = false;

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

                if(hardware.gripper.getPosition() == GripperConstants.Gripper_Grab) {
                    hardware.releaseCone();
                }
                else if (hardware.Distance()){
                    hardware.gripCone();
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
                hardware.armToStart();
                armInTimer.reset();
            }

            if(returning) {
                if(atHigh && (armInTimer.seconds() > 0)) {
                    hardware.setSlidesPower(1);
                    hardware.slidesToGround();
                    hardware.armToStart();
                    returning = false;
                    atHigh = false;
                }

                if(atMid && (armInTimer.seconds() > 0.2)) {
                    hardware.setSlidesPower(1);
                    hardware.slidesToGround();
                    hardware.armToStart();
                    hardware.armToStart();
                    returning = false;
                    atMid = false;
                }

                if(atLow && (armInTimer.seconds() > 0.3)) {
                    hardware.setSlidesPower(1);
                    hardware.slidesToGround();
                    hardware.armToStart();
                    returning = false;
                    atLow = false;
                }

                else{
                    if(armInTimer.seconds() > 0.75){
                        hardware.setSlidesPower(1);
                        hardware.slidesToGround();
                        hardware.armToStart();
                        returning = false;
                        atLow = false;
                    }
                }
            }

            // Low
            if(gamepad1.x) {
                raisingToLow = true;
                hardware.setSlidesPower(1);
                hardware.slidesToLow();
            }
            if(raisingToLow) {
                if(hardware.leftSlide.getCurrentPosition() > 600) {
                    hardware.armScoring();
                    raisingToLow = false;
                    atLow = true;
                    atMid = false;
                    atHigh = false;
                }
            }

            // Medium
            if(gamepad1.y) {
                raisingToMiddle = true;
                hardware.setSlidesPower(1);
                hardware.slidesToMedium();
            }
            if(raisingToMiddle) {
                if(hardware.leftSlide.getCurrentPosition() > 600) {
                    hardware.armScoring();
                    raisingToMiddle = false;
                    atLow = false;
                    atMid = true;
                    atHigh = false;
                }
            }

            // High
            if(gamepad1.b) {
                raisingToHigh = true;
                hardware.setSlidesPower(1);
                hardware.slidesToHigh();
            }
            if(raisingToHigh) {
                if(hardware.leftSlide.getCurrentPosition() > 600) {
                    hardware.armScoring();
                    raisingToHigh = false;
                    atLow = false;
                    atMid = false;
                    atHigh = true;
                }
            }

            // Fine Control the Slides
            if(gamepad1.dpad_down) {
                hardware.leftSlide.setTargetPosition(hardware.leftSlide.getCurrentPosition() + 50);
                hardware.rightSlide.setTargetPosition(hardware.rightSlide.getCurrentPosition() + 50);
            }
            if(gamepad1.dpad_up) {
                hardware.leftSlide.setTargetPosition(hardware.leftSlide.getCurrentPosition() - 50);
                hardware.rightSlide.setTargetPosition(hardware.rightSlide.getCurrentPosition() - 50);
            }

//            /*------- Intake -------*/
//
//            if(gamepad1.right_trigger > 0.1) {
//                hardware.intakeIn();
//            }
//
//            else if(gamepad1.right_bumper) {
//                hardware.intakeOut();
//            }
//
//            else {
//                hardware.intake(0);
//            }
//
//            if(gamepad1.dpad_left) {
//                hardware.intakeServoIn();
//            }
//
//            if(gamepad1.dpad_right) {
//                hardware.intakeServoOut();
//            }
//
//            else if (hardware.isGripping() || hardware.Distance()) {
//                hardware.intakeServoOut();
//            }
//

        }
    }



}
