package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import static org.firstinspires.ftc.teamcode.TeleOp.FieldPoseConstants.LeftAutoConstants;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.TeleOp.FieldPoseConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name = "Left_RRMidJunction")
public class Left_RRMidJunction extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Time
    public ElapsedTime timer;
    static final double FEET_PER_METER = 3.28084;

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
        Drivetrain     drivetrain    = new Drivetrain(hardwareMap);
        Gripper        gripper       = new Gripper(hardwareMap);
        Slides         slides        = new Slides(hardwareMap);
        Arm            arm           = new Arm(hardwareMap);
        Intake         intake        = new Intake(hardwareMap);

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

        timer = new ElapsedTime();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(LeftAutoConstants.startPose);


        TrajectorySequence High_Junction = drive.trajectorySequenceBuilder(LeftAutoConstants.startPose)
                .back(45)
                .splineTo(new Vector2d(LeftAutoConstants.preLoadMidJunction.getX(), LeftAutoConstants.preLoadMidJunction.getY()), LeftAutoConstants.preLoadMidJunction.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-2, slides::slidesToMedium)
                .UNSTABLE_addTemporalMarkerOffset(-1, arm::armScoring)
                .UNSTABLE_addTemporalMarkerOffset(0.3, gripper::releaseCone)
                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToCone1)
                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToStart)
                .waitSeconds(0.5)
                //Stack 1
                .splineTo(new Vector2d(LeftAutoConstants.stack.getX(), LeftAutoConstants.stack.getY()), LeftAutoConstants.stack.getHeading())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, gripper::gripCone)
                .UNSTABLE_addTemporalMarkerOffset(0.01, slides::slidesToMedium)
                .setReversed(true)
                .splineTo(new Vector2d(LeftAutoConstants.closeMidJunction.getX(), LeftAutoConstants.closeMidJunction.getY()), LeftAutoConstants.closeMidJunction.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-0.9, arm::armScoring)
                .UNSTABLE_addTemporalMarkerOffset(0, gripper::releaseCone)
                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToCone2)
                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToStart)
                .waitSeconds(0.25)
                .setReversed(false)
                //Stack 2
                .splineTo(new Vector2d(LeftAutoConstants.stack.getX()+1, LeftAutoConstants.stack.getY()), LeftAutoConstants.stack.getHeading())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, gripper::gripCone)
                .UNSTABLE_addTemporalMarkerOffset(0.01, slides::slidesToMedium)
                .setReversed(true)
                .splineTo(new Vector2d(LeftAutoConstants.closeMidJunction.getX(), LeftAutoConstants.closeMidJunction.getY()), LeftAutoConstants.closeMidJunction.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-0.9, arm::armScoring)
                .UNSTABLE_addTemporalMarkerOffset(0, gripper::releaseCone)
                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToCone3)
                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToStart)
                .waitSeconds(0.25)
                .setReversed(false)
                //Stack 3
                .splineTo(new Vector2d(LeftAutoConstants.stack.getX()+2, LeftAutoConstants.stack.getY()), LeftAutoConstants.stack.getHeading())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, gripper::gripCone)
                .UNSTABLE_addTemporalMarkerOffset(0.01, slides::slidesToMedium)
                .setReversed(true)
                .splineTo(new Vector2d(LeftAutoConstants.closeMidJunction.getX(), LeftAutoConstants.closeMidJunction.getY()), LeftAutoConstants.closeMidJunction.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-0.9, arm::armScoring)
                .UNSTABLE_addTemporalMarkerOffset(0, gripper::releaseCone)
                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToCone4)
                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToStart)
                .waitSeconds(0.25)
                .setReversed(false)
                //Stack 4
                .splineTo(new Vector2d(LeftAutoConstants.stack.getX()+3, LeftAutoConstants.stack.getY()), LeftAutoConstants.stack.getHeading())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, gripper::gripCone)
                .UNSTABLE_addTemporalMarkerOffset(0.01, slides::slidesToMedium)
                .setReversed(true)
                .splineTo(new Vector2d(LeftAutoConstants.closeMidJunction.getX(), LeftAutoConstants.closeMidJunction.getY()), LeftAutoConstants.closeMidJunction.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(-0.9, arm::armScoring)
                .UNSTABLE_addTemporalMarkerOffset(0, gripper::releaseCone)
                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToCone5)
                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToStart)
                .waitSeconds(0.25)
                .setReversed(false)
                .splineTo(new Vector2d(LeftAutoConstants.middleTile.getX(), LeftAutoConstants.middleTile.getY()), LeftAutoConstants.middleTile.getHeading())
                .build();

        Trajectory trajLeft = drive.trajectoryBuilder(High_Junction.end())
                .forward(24)
                .build();

        Trajectory trajRight = drive.trajectoryBuilder(High_Junction.end())
                .back(24)
                .build();

        /* OpMode */
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
            gripper.gripCone();
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

//        gripper.gripCone();
        intake.intakeServoOut();
        drive.followTrajectorySequence(High_Junction);

        if (tagOfInterest == null || tagOfInterest.id == Middle) {
            // Do nothing

        } else if (tagOfInterest.id == Left) {
            // Left Code
            drive.followTrajectory(trajLeft);


        } else if (tagOfInterest.id == Right) {
            // Right Code
            drive.followTrajectory(trajRight);

        } else {
            // Do nothing
        }

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        for (String s : Arrays.asList(String.format("\nDetected tag ID=%d", detection.id), String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER), String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER), String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER), String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)), String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)), String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)))) {
            telemetry.addLine(s);
        }
    }
}
