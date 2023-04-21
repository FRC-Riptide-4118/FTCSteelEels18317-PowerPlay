package org.firstinspires.ftc.teamcode.Auto.WorldsAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Alignment;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.TeleOp.FieldPoseConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous(group = "Worlds")
public class Left_AutoSafeMedium extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public static double DELAY_TIME_SECONDS = 2.0;

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

    public static double inchesToStack = 24.25;
    public static double slidesArmDelay = 0.50;
    public static double wristIntakeArmDelay = 0.25;


    @Override
    public void runOpMode() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        //Hardware Classes
        Gripper     gripper     = new Gripper(hardwareMap);
        Arm         arm         = new Arm(hardwareMap);
        Wrist       wrist       = new Wrist(hardwareMap);
        Intake      intake      = new Intake(hardwareMap);
        Alignment   alignment   = new Alignment(hardwareMap);
        Slides      slides      = new Slides(hardwareMap);

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
        SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);


        mecanumDrive.setPoseEstimate(FieldPoseConstants.LeftAutoConstants.startPose);

        TrajectorySequence safeSequence = mecanumDrive.trajectorySequenceBuilder(mecanumDrive.getPoseEstimate())
                    .UNSTABLE_addTemporalMarkerOffset(0.0, arm::armToMiddle)
                .waitSeconds(DELAY_TIME_SECONDS)
                .lineToConstantHeading(FieldPoseConstants.LeftAutoConstants.allyStartTile)
                .forward(24)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, slides::slidesToMedium)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, wrist::toScoring)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, arm::armToPreScore)
                .lineToLinearHeading(FieldPoseConstants.LeftAutoConstants.allyScoreCone)
                .back(5)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, alignment::score)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, arm::armToScore)
                    .UNSTABLE_addTemporalMarkerOffset(0.2, gripper::releaseCone)
                    .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
                    .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
                    .UNSTABLE_addTemporalMarkerOffset(0.6, arm::armToStart)
                    .UNSTABLE_addTemporalMarkerOffset(0.6, alignment::up)
                    .UNSTABLE_addTemporalMarkerOffset(0.7, slides::slidesToStart)
                .forward(5)
                .build();

        TrajectorySequence parkLeft = mecanumDrive.trajectorySequenceBuilder(safeSequence.end())
                .lineToLinearHeading(FieldPoseConstants.LeftAutoConstants.parkAllyLeftSafe)
                .build();

        TrajectorySequence parkMiddle = mecanumDrive.trajectorySequenceBuilder(safeSequence.end())
                .lineToLinearHeading(FieldPoseConstants.LeftAutoConstants.parkAllyMiddleSafe)
                .build();

        TrajectorySequence parkRight = mecanumDrive.trajectorySequenceBuilder(safeSequence.end())
                .lineToLinearHeading(FieldPoseConstants.LeftAutoConstants.parkAllyRightSafe)
                .build();


        /* OpMode */
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

            gripper.gripCone();
            intake.open();
//            telemetry.update();
//            sleep(20);
        }

        slides.resetEncoders();

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

        mecanumDrive.followTrajectorySequence(safeSequence);

        if (tagOfInterest == null || tagOfInterest.id == Middle) {
            mecanumDrive.followTrajectorySequence(parkMiddle);

        } else if (tagOfInterest.id == Left) {
            mecanumDrive.followTrajectorySequence(parkLeft);

        } else if (tagOfInterest.id == Right) {
            mecanumDrive.followTrajectorySequence(parkRight);

        } else {
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
