package org.firstinspires.ftc.teamcode.BlakeStuff;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.OpenCV.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Disabled
@Autonomous(name = "(Blake) Test AprilTag Parking", group = "Blake")
public class BlakeTestAprilTagParking extends LinearOpMode {

    BlakeRobotHardware hardware;

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
    int tagIDLeft = 1;
    int tagIDMiddle = 2;
    int tagIDRight = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        hardware = new BlakeRobotHardware(hardwareMap);
        hardware.init();

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

        // Loop while waiting for start
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == tagIDLeft || tag.id == tagIDMiddle || tag.id == tagIDRight)
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


        // Start of op mode (driver has pressed "play")

        // Show found tag on telemetry, if any
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

        if (tagOfInterest.id == tagIDLeft)
        {
            driveInches(26, 0.3);
            strafeInches(-25, 0.3);
            driveInches(5, 0.3);
        }
        else if (tagOfInterest == null || tagOfInterest.id == tagIDMiddle)
        {
            driveInches(30, 0.3);
        }
        else if (tagOfInterest.id == tagIDRight)
        {
            driveInches(26, 0.3);
            strafeInches(27, 0.3);
            driveInches(5, 0.3);
        }
    }


    /**
     * Drive straight forward a set number of inches.
     *
     * @param inches the distance to drive forward
     * @param power the drive motor power to use
     */
    public void driveInches(double inches, double power)
    {
        drivetrain(inches, inches, inches, inches, power);
    }

    /**
     * Strafe sideways a set number of inches.
     *
     * @param inches the distance to strafe to the right
     * @param power the drive motor power to use
     */
    public void strafeInches(double inches, double power)
    {
        drivetrain(inches, -inches, -inches, inches, power);
    }

    /**
     * Set drivetrain motors to move the requested number of inches.
     *
     * @param frontLeftInches inches to move for the front left drive motor
     * @param frontRightInches inches to move for the front right drive motor
     * @param rearLeftInches inches to move for the rear left drive motor
     * @param rearRightInches inches to move for the rear right drive motor
     * @param power the drive motor power to use
     */
    public void drivetrain(double frontLeftInches, double frontRightInches, double rearLeftInches, double rearRightInches, double power) {

        // Set targets
        hardware.frontRight.setTargetPosition(hardware.frontRight.getCurrentPosition() + hardware.inchesToCounts(frontRightInches));
        hardware.frontLeft.setTargetPosition(hardware.rearRight.getCurrentPosition() + hardware.inchesToCounts(rearRightInches));
        hardware.rearRight.setTargetPosition(hardware.frontLeft.getCurrentPosition() + hardware.inchesToCounts(frontLeftInches));
        hardware.rearLeft.setTargetPosition(hardware.rearLeft.getCurrentPosition() + hardware.inchesToCounts(rearLeftInches));

        // Set to run to position
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power to begin moving
        hardware.frontLeft.setPower(power);
        hardware.rearLeft.setPower(power);
        hardware.rearRight.setPower(power);
        hardware.frontRight.setPower(power);

        // Wait for motors to finish and print telemetry in the meantime
        while (opModeIsActive() && (/*frontRight.isBusy()/* ||*/ hardware.frontLeft.isBusy()) && (hardware.rearRight.isBusy() /*|| rearLeft.isBusy()*/)) {
            // Color Sensor
            telemetry.addData("front left pos", hardware.frontLeft.getCurrentPosition());
            telemetry.addData("front right pos", hardware.frontRight.getCurrentPosition());
            telemetry.addData("back left pos", hardware.rearLeft.getCurrentPosition());
            telemetry.addData("back right pos", hardware.rearRight.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("front left power", hardware.frontLeft.getPower());
            telemetry.addData("front right power", hardware.frontRight.getPower());
            telemetry.addData("back left power", hardware.rearLeft.getPower());
            telemetry.addData("back right power", hardware.rearRight.getPower());
            telemetry.addLine();
            telemetry.addData("front left target", hardware.frontLeft.getTargetPosition());
            telemetry.addData("front right target", hardware.frontRight.getTargetPosition());
            telemetry.addData("back left target", hardware.rearLeft.getTargetPosition());
            telemetry.addData("back right target", hardware.rearRight.getTargetPosition());
            telemetry.update();
        }

        // Stop motors
        hardware.frontLeft.setPower(0);
        hardware.rearLeft.setPower(0);
        hardware.rearRight.setPower(0);
        hardware.frontRight.setPower(0);
    }

    /**
     * Provide telemetry on a detected AprilTag.
     *
     * @param detection the detected AprilTag
     */
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

    /**
     * Score a preloaded cone.
     */
    private void scorePreloadedCone() {
        driveInches(26, 0.3);
        strafeInches(-5, 0.3);
        sleep(200); // FIXME why is this in here?
        strafeInches(5, 0.3);
        driveInches(-52, 0.3);
    }


}
