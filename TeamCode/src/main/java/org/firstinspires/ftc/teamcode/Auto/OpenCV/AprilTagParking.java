/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Auto.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "AprilTag")
public class AprilTagParking extends LinearOpMode {
    private DcMotor frontLeft  = null;
    private DcMotor  rearRight  = null;
    private DcMotor  frontRight  = null;
    private DcMotor  rearLeft  = null;

    int frontRightTarget;
    int frontLeftTarget;
    int rearRightTarget;
    int rearLeftTarget;
    double DRIVE_COUNTS_PER_IN;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag IDs of Sleeve
    int Left = 1;
    int Middle = 2;
    int Right = 3;

    AprilTagDetection tagOfInterest = null;

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

        HD_COUNTS_PER_REV = 28;
        DRIVE_GEAR_REDUCTION = 20;
        WHEEL_CIRCUMFERENCE_MM = 101.6 * Math.PI;
        DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
        DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Left || tag.id == Middle || tag.id == Right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /***Code***/
        
        if (tagOfInterest.id == Left){
            // Left Code
            drivetrain(22, 22, 22, 22, .3, telemetry);
            Reset_Encoders();
            drivetrain(-22, 22, -22, 22, .3, telemetry);
            Reset_Encoders();
        }
        else if (tagOfInterest == null || tagOfInterest.id == Middle) {
            // Middle Code
            drivetrain(22, 22, 22, 22, .3, telemetry);
            Reset_Encoders();
        }
        else if (tagOfInterest.id == Right) {
            // Right Code
            drivetrain(22, 22, 22, 22, .3, telemetry);
            Reset_Encoders();
            drivetrain(22, -22, 22, -22, .3, telemetry);
            Reset_Encoders();
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }



//Resetting Encoders
  private void Reset_Encoders() {
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftTarget = 0;
    frontRightTarget = 0;
    rearRightTarget = 0;
    rearLeftTarget = 0;
  }

  // InchesToCounts
  public int inchesToCounts(double inches) {
    return (int) (inches * DRIVE_COUNTS_PER_IN);
  }

//Inches
  private void drivetrain(double frontLeftInches, double frontRightInches, double rearLeftInches, double rearRightInches, double Power, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
    if (opModeIsActive()) {
      frontRightTarget = frontRight.getCurrentPosition() + inchesToCounts(frontRightInches);
      rearRightTarget = rearRight.getCurrentPosition() + inchesToCounts(rearRightInches);
      frontLeftTarget = frontLeft.getCurrentPosition() + inchesToCounts(frontLeftInches);
      rearLeftTarget = rearLeft.getCurrentPosition() + inchesToCounts(rearLeftInches);

      frontRight.setTargetPosition(frontRightTarget);
      frontLeft.setTargetPosition(frontLeftTarget);
      rearRight.setTargetPosition(rearRightTarget);
      rearLeft.setTargetPosition(rearLeftTarget);

      frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      frontLeft.setPower(Power);
      rearLeft.setPower(Power);
      rearRight.setPower(Power);
      frontRight.setPower(Power); 
    }
  }
}
