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
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.DrivetrainTeleOp;
import org.firstinspires.ftc.teamcode.Commands.GripperTeleOp;
import org.firstinspires.ftc.teamcode.Commands.ScoringTeleOp;
import org.firstinspires.ftc.teamcode.Commands.IntakeTeleOp;
import org.firstinspires.ftc.teamcode.Commands.IntakeServosTeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeServos;

@TeleOp(name = "TeleOpMecanumDrive", group = "Robot")

public class TeleOpCommandsTesting extends LinearOpMode {


    @Override
    public void runOpMode() {

        GripperTeleOp gripperTeleOp             = new GripperTeleOp(new Gripper(hardwareMap), gamepad1);
        DrivetrainTeleOp drivetrainTeleOp       = new DrivetrainTeleOp(new Drivetrain(hardwareMap), gamepad1);
        ScoringTeleOp scoringTeleOp             = new ScoringTeleOp(new Arm(hardwareMap), new Slides(hardwareMap), gamepad1);
        IntakeTeleOp intakeTeleOp               = new IntakeTeleOp(new Intake(hardwareMap), new Gripper(hardwareMap), gamepad1);
        IntakeServosTeleOp intakeServosTeleOp   = new IntakeServosTeleOp(new IntakeServos(hardwareMap), new Gripper(hardwareMap), gamepad1);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver1");
        telemetry.update();

        //Hardware Classes
        Drivetrain   drivetrain    = new Drivetrain(hardwareMap);
        Gripper      gripper       = new Gripper(hardwareMap);
        Slides       slides        = new Slides(hardwareMap);
        Intake       intake        = new Intake(hardwareMap);
        IntakeServos intakeServos  = new IntakeServos(hardwareMap);

        intakeServos.getSubsystem();
        intake.getSubsystem();
        slides.getSubsystem();
        gripper.getSubsystem();
        drivetrain.getSubsystem();

        gripperTeleOp.initialize();
        intakeTeleOp.initialize();
        scoringTeleOp.initialize();
        intakeServosTeleOp.initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*-------Drivetrain-------*/
            // Gamepad controls
            drivetrainTeleOp.execute();

            /*-------Gripper-------*/
            gripperTeleOp.execute();

            /*-------Lift & Arm-------*/
            scoringTeleOp.execute();

            // Fine Control the Slides
//            if(gamepad1.dpad_down) {
//                leftSlidePrevPos = hardware.leftSlide.getTargetPosition();
//                hardware.leftSlide.setTargetPosition(hardware.leftSlide.getCurrentPosition() + 50);
//                hardware.rightSlide.setTargetPosition(hardware.rightSlide.getCurrentPosition() + 50);
//            }
//            if(gamepad1.dpad_up) {
//                leftSlidePrevPos = hardware.leftSlide.getTargetPosition();
//                hardware.leftSlide.setTargetPosition(hardware.leftSlide.getCurrentPosition() - 50);
//                hardware.rightSlide.setTargetPosition(hardware.rightSlide.getCurrentPosition() - 50);
//            }

            /*------- Intake -------*/
            intakeTeleOp.execute();
            intakeServosTeleOp.execute();

        }
    }



}

