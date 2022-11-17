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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Slides;


@TeleOp(name = "Testing", group = "Robot")

public class Testing extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor Slides = null;
    public DcMotor frontLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontRight = null;
    public DcMotor rearLeft = null;
    public Servo Gripper = null;
    public DcMotor arm = null;

    private static final int Slides_Start = 0;
    private static final int Arm_Start = -5;
    private static final int Slides_Low = -750;
    private static final int Arm_Low = 350;
    private static final int Slides_Medium = -950;
    private static final int Arm_Medium = 350;
    private static final int Slides_High = -1050;
    private static final int Arm_High = 350;
    private static final double Gripper_Release = 0.9;
    private static final double Gripper_Grab = 0;


    @Override
    public void runOpMode() {
        // Define and Initialize Motors
        Slides slides = new Slides(hardwareMap);
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_wheel");
        rearLeft = hardwareMap.get(DcMotor.class, "rear_left_wheel");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_wheel");
        rearRight = hardwareMap.get(DcMotor.class, "rear_right_wheel");
        Gripper = hardwareMap.get(Servo.class, "left_intake");
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Reversing the motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        // Reset the slides
        Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Slides.setTargetPosition(Slides_Start);
        arm.setTargetPosition(Arm_Start);

        Slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*-------Drivetrain-------*/
            // Gamepad controls
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            frontLeft.setPower(v1);
            frontRight.setPower(v2);
            rearLeft.setPower(v3);
            rearRight.setPower(v4);

            /*-------INTAKE-------*/
            if (gamepad1.left_bumper) {
                Gripper.setPosition(Gripper_Grab);
            }
            if (gamepad1.right_bumper) {
                Gripper.setPosition(Gripper_Release);
            }


            // Lift & Arm
            if (gamepad1.a){
                arm.setPower(.4);
                arm.setTargetPosition(Arm_Start);
                sleep(750);
                Slides.setPower(.1);
                Slides.setTargetPosition(Slides_Start);
                sleep(500);
                Gripper.setPosition(Gripper_Release);
            }
            if (gamepad1.x) {
                Slides.setPower(.5);
                Slides.setTargetPosition(Slides_Low);
                sleep(500);
                arm.setPower(.5);
                arm.setTargetPosition(Arm_Low);
            }
            if (gamepad1.y) {
                Gripper.setPosition(Gripper_Grab);
                Slides.setPower(.5);
                Slides.setTargetPosition(Slides_Medium);
                sleep(500);
                arm.setPower(.5);
                arm.setTargetPosition(Arm_Medium);
            }
            /* if (gamepad1.b) {
                Gripper.setPosition(Gripper_Grab);
                Slides.setPower(.5);
                Slides.setTargetPosition(Slides_High);
                sleep(500);
                arm.setPower(.5);
                arm.setTargetPosition(Arm_High);
            }  */
            slides.periodic();
        }
    }
}

