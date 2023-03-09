///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.GripperConstants;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.EelverHardware;
//import org.firstinspires.ftc.teamcode.Subsystems.Slides;
//
//@TeleOp(group = "Robot")
//public class TeleOpStateMachine extends LinearOpMode {
//
//    private enum GripperState
//    {
//        CLOSED,
//        OPEN
//    }
//    private enum ArmState
//    {
//        START,
//        SCORING
//    }
//    private enum SlidesState
//    {
//        GROUND,
//        LOW,
//        MEDIUM,
//        HIGH
//    }
//
//    private boolean lowering = false;
//
//    GripperState    gripState   = GripperState.CLOSED;
//    ArmState        armState    = ArmState.START;
//    SlidesState     slidesState = SlidesState.GROUND;
//
//    SlidesState     prevSlidesState = SlidesState.GROUND;
//
//
//
//    private ElapsedTime armInTimer;
//    private ElapsedTime testTimer;
//
//    private boolean pressedLastIteration = false;
//
//    @Override
//    public void runOpMode() {
//        EelverHardware hardware = new EelverHardware();
//        hardware.init(hardwareMap);
//
//        armInTimer = new ElapsedTime();
//        testTimer = new ElapsedTime();
//        armInTimer.reset();
//        testTimer.reset();
//
//        telemetry.addLine("Ready");
//        telemetry.update();
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            /*-------Drivetrain-------*/
//            // Gamepad controls
//            double drive = -gamepad1.left_stick_y;
//            double strafe = gamepad1.left_stick_x;
//            double twist = gamepad1.right_stick_x;
//            double slowMode = Math.abs(1.25 - gamepad1.left_trigger);
//
//            hardware.setMecanumPower(drive, strafe, twist, slowMode);
//
//            /*-------Gripper-------*/
//            boolean pressed = gamepad1.left_bumper;
//            if (pressed & !pressedLastIteration) {
//                if(gripState == GripperState.CLOSED) hardware.releaseCone();
//                else hardware.gripCone();
//            }
//            pressedLastIteration = pressed;
//
//            /*-------Lift & Arm-------*/
//            hardware.setSlidesPower(1);
//            prevSlidesState = slidesState;
//            // Ground
//            if(gamepad1.a) {
//                slidesState = SlidesState.GROUND;
//                hardware.armToStart();
//                armInTimer.reset();
//            }
//            else if(gamepad1.x) {
//                slidesState = SlidesState.LOW;
//            }
//            else if(gamepad1.y) {
//                slidesState = SlidesState.MEDIUM;
//            }
//            else if(gamepad1.b) {
//                slidesState = SlidesState.HIGH;
//            }
//
////            lowering =
////                    prevSlidesState == SlidesState.HIGH &&
////                            (slidesState == SlidesState.MEDIUM || slidesState == SlidesState.LOW || slidesState == SlidesState.GROUND) ||
////                    prevSlidesState == SlidesState.MEDIUM &&
////                            (slidesState == SlidesState.LOW || slidesState == SlidesState.GROUND) ||
////                    prevSlidesState == HIGH
////            ;
//
//            if(lowering) {
//                if(atHigh && (armInTimer.seconds() > 0)) {
//                    hardware.setSlidesPower(1);
//                    hardware.slidesToGround();
//                    hardware.armToStart();
//                    returning = false;
//                    atHigh = false;
//                }
//
//                if(atMid && (armInTimer.seconds() > 0.2)) {
//                    hardware.setSlidesPower(1);
//                    hardware.slidesToGround();
//                    hardware.armToStart();
//                    hardware.armToStart();
//                    returning = false;
//                    atMid = false;
//                }
//
//                if(atLow && (armInTimer.seconds() > 0.3)) {
//                    hardware.setSlidesPower(1);
//                    hardware.slidesToGround();
//                    hardware.armToStart();
//                    returning = false;
//                    atLow = false;
//                }
//
//                else{
//                    if(armInTimer.seconds() > 0.75){
//                        hardware.setSlidesPower(1);
//                        hardware.slidesToGround();
//                        hardware.armToStart();
//                        returning = false;
//                        atLow = false;
//                    }
//                }
//            }
//
//            if(raisingToLow) {
//                if(hardware.leftSlide.getCurrentPosition() > 600) {
//                    hardware.armScoring();
//                }
//            }
//
//            // Medium
//            if(raisingToMiddle) {
//                if(hardware.leftSlide.getCurrentPosition() > 600) {
//                    hardware.armScoring();
//                }
//            }
//
//            // High
//            if(raisingToHigh) {
//                if(hardware.leftSlide.getCurrentPosition() > 600) {
//                    hardware.armScoring();
//                }
//            }
//        }
//    }
//}
