package org.firstinspires.ftc.teamcode.Auto.WorldsAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Util.FieldPoseConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Vector;

@Config
@Autonomous(group = "Worlds")
public class AutoWorldsDelayedParkClose extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Time
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

    // Constants
    public static double delaySeconds = 2.0;
    public static double forwardDriveInches = 28.0;


    @Override
    public void runOpMode()
    {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);

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


        mecanumDrive.setPoseEstimate(FieldPoseConstants.LeftAutoConstants.startPose);

        TrajectorySequence driveForward = mecanumDrive.trajectorySequenceBuilder(FieldPoseConstants.LeftAutoConstants.startPose)
                .waitSeconds(delaySeconds)
                .forward(forwardDriveInches)
                .build();


        TrajectorySequence parkLeft = mecanumDrive.trajectorySequenceBuilder(driveForward.end())
                .lineToConstantHeading(new Vector2d(
                        FieldPoseConstants.LeftAutoConstants.parkCloseLeft.getX(),
                        FieldPoseConstants.LeftAutoConstants.parkCloseLeft.getY()))
                .build();

        TrajectorySequence parkMiddle = mecanumDrive.trajectorySequenceBuilder(driveForward.end())
                .lineToConstantHeading(new Vector2d(
                        FieldPoseConstants.LeftAutoConstants.parkCloseLeft.getX(),
                        FieldPoseConstants.LeftAutoConstants.parkCloseLeft.getY()))
                .build();

        TrajectorySequence parkRight = mecanumDrive.trajectorySequenceBuilder(driveForward.end())
                .lineToConstantHeading(new Vector2d(
                        FieldPoseConstants.LeftAutoConstants.parkCloseLeft.getX(),
                        FieldPoseConstants.LeftAutoConstants.parkCloseLeft.getY()))
                .build();

        while (opModeInInit())
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

        mecanumDrive.followTrajectorySequence(driveForward);

        if (tagOfInterest == null || tagOfInterest.id == Middle)
        {
            mecanumDrive.followTrajectorySequence(parkMiddle);
        }
        else if (tagOfInterest.id == Left)
        {
            mecanumDrive.followTrajectorySequence(parkLeft);
        }
        else if (tagOfInterest.id == Right)
        {
            mecanumDrive.followTrajectorySequence(parkRight);
        }
        else
        {
            mecanumDrive.followTrajectorySequence(parkMiddle);
        }

        while(opModeIsActive());
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        for (String s : Arrays.asList(String.format("\nDetected tag ID=%d", detection.id), String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER), String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER), String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER), String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)), String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)), String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)))) {
            telemetry.addLine(s);
        }
    }

}
