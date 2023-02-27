package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import java.util.ArrayList;
@Disabled
@Autonomous(name = "Left_CloseHighJunction")
public class Left_CloseHighJunction extends LinearOpMode {

    // Time
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Time
    public ElapsedTime timer;
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

        //Hardware Classes
        Drivetrain   drivetrain    = new Drivetrain(hardwareMap);
        Gripper      gripper       = new Gripper(hardwareMap);
        Slides       slides        = new Slides(hardwareMap);
        Arm          arm           = new Arm(hardwareMap);
        Intake       intake        = new Intake(hardwareMap);

        intake.getSubsystem();
        slides.getSubsystem();
        arm.getSubsystem();
        gripper.getSubsystem();
        drivetrain.getSubsystem();

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

        timer = new ElapsedTime();

        SampleMecanumDrive drive  = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose          = new Pose2d(37, 65, Math.toRadians(90));
        Pose2d closeHighJunction  = new Pose2d(31, 8, Math.toRadians(225));
        Pose2d Stack              = new Pose2d(65, 16, Math.toRadians(3));
        Pose2d Middle_Tile        = new Pose2d(40,18, Math.toRadians(0));


        drive.setPoseEstimate(startPose);

        TrajectorySequence High_Junction = drive.trajectorySequenceBuilder(startPose)
                .back(35)
                //Pre-Load
                .splineTo(new Vector2d(closeHighJunction.getX(), closeHighJunction.getY()), closeHighJunction.getHeading())
                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(-1, slides::slidesToHigh)
//                .UNSTABLE_addTemporalMarkerOffset(0.1, gripper::releaseCone)
//                .UNSTABLE_addTemporalMarkerOffset(1.2, arm::armToStart)
//                .UNSTABLE_addTemporalMarkerOffset(1.4, slides::slidesToStart)
//                .UNSTABLE_addTemporalMarkerOffset(1.0, gripper::releaseCone)
                //Stack 1
                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-0.9, arm::armToCone1)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, gripper::gripCone)
                .UNSTABLE_addTemporalMarkerOffset(0.01, slides::slidesToLow)
                .UNSTABLE_addTemporalMarkerOffset(-.8, slides::slidesToStart)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(closeHighJunction.getX(), closeHighJunction.getY()), closeHighJunction.getHeading())
                .waitSeconds(0.5)
                .setReversed(false)
                //Stack 2
                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-.9, arm::armToCone2)
                .waitSeconds(0.5)
                .setReversed(true)
                .splineTo(new Vector2d(closeHighJunction.getX(), closeHighJunction.getY()), closeHighJunction.getHeading())
                .waitSeconds(0.5)
                .setReversed(false)
                //Stack 3
                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-.9, arm::armToCone3)
                .waitSeconds(0.5)
                .setReversed(true)
                .splineTo(new Vector2d(closeHighJunction.getX(), closeHighJunction.getY()), closeHighJunction.getHeading())
                .waitSeconds(0.5)
                .setReversed(false)
                //Stack 4
                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-.9, arm::armToCone4)
                .waitSeconds(0.5)
                .setReversed(true)
                .splineTo(new Vector2d(closeHighJunction.getX(), closeHighJunction.getY()), closeHighJunction.getHeading())
                .waitSeconds(0.5)
                .setReversed(false)
                //Back to Stack
                .splineTo(new Vector2d(Middle_Tile.getX(), Middle_Tile.getY()), Middle_Tile.getHeading())
                .build();

        Trajectory trajLeft = drive.trajectoryBuilder(High_Junction.end())
                .forward(24)
                .build();

        Trajectory trajRight = drive.trajectoryBuilder(High_Junction.end())
                .back(24)
                .build();

        gripper.gripCone();
        arm.armWiggle();
        intake.intakeServoOut();
        gripper.releaseCone();

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

        if (opModeIsActive()) {
            drive.followTrajectorySequence(High_Junction);
            if (tagOfInterest.id == Left) {
                // Left Code
                drive.followTrajectory(trajLeft);

            } else if (tagOfInterest == null || tagOfInterest.id == Middle) {
                // Middle Code

            } else if (tagOfInterest.id == Right) {
                // Right Code
                drive.followTrajectory(trajRight);

            } else {

            }
        }
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

}
