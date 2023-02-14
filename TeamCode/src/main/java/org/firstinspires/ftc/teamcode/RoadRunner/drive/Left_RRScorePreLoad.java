package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.EelverHardware;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Left_RRScorePreLoaded")
public class Left_RRScorePreLoad extends LinearOpMode {

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

        EelverHardware hardware = new EelverHardware();

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
        hardware.init(hardwareMap); // Edited by Blake Sanders 1/31/22
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose          = new Pose2d(37, 65, Math.toRadians(90));
        Pose2d closeHighJunction  = new Pose2d(31, 8, Math.toRadians(225));
        Pose2d Stack              = new Pose2d(63, 16, Math.toRadians(3));
        drive.setPoseEstimate(startPose);

        TrajectorySequence High_Junction = drive.trajectorySequenceBuilder(startPose)
                .back(35)
                //Pre-Load
                .splineTo(new Vector2d(closeHighJunction.getX(), closeHighJunction.getY()), closeHighJunction.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(1, hardware::releaseCone)
                .waitSeconds(0.5)
                //Stack 1
                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                .waitSeconds(0.5)
                .setReversed(true)
                .splineTo(new Vector2d(closeHighJunction.getX(), closeHighJunction.getY()), closeHighJunction.getHeading())
                .waitSeconds(0.5)
                .setReversed(false)
                //Stack 2
                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                .waitSeconds(0.5)
                .setReversed(true)
                .splineTo(new Vector2d(closeHighJunction.getX(), closeHighJunction.getY()), closeHighJunction.getHeading())
                .waitSeconds(0.5)
                .setReversed(false)
                //Stack 3
                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                .waitSeconds(0.5)
                .setReversed(true)
                .splineTo(new Vector2d(closeHighJunction.getX(), closeHighJunction.getY()), closeHighJunction.getHeading())
                .waitSeconds(0.5)
                .setReversed(false)
                //Stack 4
                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                .waitSeconds(0.5)
                .setReversed(true)
                .splineTo(new Vector2d(closeHighJunction.getX(), closeHighJunction.getY()), closeHighJunction.getHeading())
                .waitSeconds(0.5)
                .setReversed(false)
                //Stack 5
                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                .waitSeconds(0.5)
                .setReversed(true)
                .splineTo(new Vector2d(closeHighJunction.getX(), closeHighJunction.getY()), closeHighJunction.getHeading())
                .waitSeconds(0.5)
                .setReversed(false)
                //Back to Stack
                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                .waitSeconds(0.5)
                .build();

        hardware.gripCone();
        hardware.armWiggle();

        waitForStart();

//        while (!isStarted() && !isStopRequested())
//        {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if(currentDetections.size() != 0)
//            {
//                boolean tagFound = false;
//
//                for(AprilTagDetection tag : currentDetections)
//                {
//                    if(tag.id == Left || tag.id == Middle || tag.id == Right)
//                    {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
//                if(tagFound)
//                {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                }
//                else
//                {
//                    telemetry.addLine("Don't see tag of interest :(");
//
//                    if(tagOfInterest == null)
//                    {
//                        telemetry.addLine("(The tag has never been seen)");
//                    }
//                    else
//                    {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            }
//            else
//            {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("(The tag has never been seen)");
//                }
//                else
//                {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }
//
//            telemetry.update();
//            sleep(20);
//        }

        drive.followTrajectorySequence(High_Junction);
    }

//    void tagToTelemetry(AprilTagDetection detection)
//    {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }

}
