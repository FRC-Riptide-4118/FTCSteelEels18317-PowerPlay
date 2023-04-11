package org.firstinspires.ftc.teamcode.Auto.WorldsAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Subsystems.Alignment;
import org.firstinspires.ftc.teamcode.TeleOp.FieldPoseConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.text.FieldPosition;
import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous(group = "Worlds")
public class AutoWorldsMid extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

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


        TrajectorySequence testSequence = mecanumDrive.trajectorySequenceBuilder(FieldPoseConstants.LeftAutoConstants.startPose)
                .UNSTABLE_addTemporalMarkerOffset(-7, arm::armToMiddle)
                .forward(55)
                .setReversed(true)
                .splineTo(new Vector2d(FieldPoseConstants.LeftAutoConstants.closeMidJunction.getX(), FieldPoseConstants.LeftAutoConstants.closeMidJunction.getY()), FieldPoseConstants.LeftAutoConstants.closeMidJunction.getHeading())
                // PreLoad
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-3, intake::open)
                .UNSTABLE_addTemporalMarkerOffset(-3, gripper::gripCone)
                .UNSTABLE_addTemporalMarkerOffset(-2.5, alignment::score)
                .UNSTABLE_addTemporalMarkerOffset(-1.8, arm::armToSCORE)
                .UNSTABLE_addTemporalMarkerOffset(-1.75, slides::slidesToMedium)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, wrist::toScoring)
                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesDrop)
                .UNSTABLE_addTemporalMarkerOffset(0, arm::armToDrop)
                .UNSTABLE_addTemporalMarkerOffset(0.1, gripper::releaseCone)
                .UNSTABLE_addTemporalMarkerOffset(0.3, slides::slidesUp)
                .UNSTABLE_addTemporalMarkerOffset(0.4, slides::slidesToCone5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, wrist::toStart)
                .UNSTABLE_addTemporalMarkerOffset(0.5, arm::armToStart)
                // Cone1
                .setReversed(false)
                .splineTo(new Vector2d(FieldPoseConstants.LeftAutoConstants.stack.getX(), FieldPoseConstants.LeftAutoConstants.stack.getY()), FieldPoseConstants.LeftAutoConstants.stack.getHeading())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, gripper::gripCone)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, slides::slidesToMedium)
                .UNSTABLE_addTemporalMarkerOffset(0.11, arm::armToSCORE)
                .UNSTABLE_addTemporalMarkerOffset(0.11, wrist::toScoring)
                .setReversed(true)
                .splineTo(new Vector2d(FieldPoseConstants.LeftAutoConstants.closeMidJunction.getX(), FieldPoseConstants.LeftAutoConstants.closeMidJunction.getY()), FieldPoseConstants.LeftAutoConstants.closeMidJunction.getHeading())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, slides::slidesDrop)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, arm::armToDrop)
                .UNSTABLE_addTemporalMarkerOffset(-0.7, gripper::releaseCone)
                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::armToStart)
                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesToCone4)
                // Cone2
                .setReversed(false)
                .splineTo(new Vector2d(FieldPoseConstants.LeftAutoConstants.stack.getX(), FieldPoseConstants.LeftAutoConstants.stack.getY()), FieldPoseConstants.LeftAutoConstants.stack.getHeading())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, gripper::gripCone)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, slides::slidesToMedium)
                .UNSTABLE_addTemporalMarkerOffset(0.11, arm::armToSCORE)
                .UNSTABLE_addTemporalMarkerOffset(0.11, wrist::toScoring)
                .setReversed(true)
                .splineTo(new Vector2d(FieldPoseConstants.LeftAutoConstants.closeMidJunction.getX(), FieldPoseConstants.LeftAutoConstants.closeMidJunction.getY()), FieldPoseConstants.LeftAutoConstants.closeMidJunction.getHeading())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, slides::slidesDrop)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, arm::armToDrop)
                .UNSTABLE_addTemporalMarkerOffset(-0.7, gripper::releaseCone)
                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::armToStart)
                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesToCone4)
                // Cone3
                .setReversed(false)
                .splineTo(new Vector2d(FieldPoseConstants.LeftAutoConstants.stack.getX(), FieldPoseConstants.LeftAutoConstants.stack.getY()), FieldPoseConstants.LeftAutoConstants.stack.getHeading())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, gripper::gripCone)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, slides::slidesToMedium)
                .UNSTABLE_addTemporalMarkerOffset(0.11, arm::armToSCORE)
                .UNSTABLE_addTemporalMarkerOffset(0.11, wrist::toScoring)
                .setReversed(true)
                .splineTo(new Vector2d(FieldPoseConstants.LeftAutoConstants.closeMidJunction.getX(), FieldPoseConstants.LeftAutoConstants.closeMidJunction.getY()), FieldPoseConstants.LeftAutoConstants.closeMidJunction.getHeading())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, slides::slidesDrop)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, arm::armToDrop)
                .UNSTABLE_addTemporalMarkerOffset(-0.7, gripper::releaseCone)
                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::armToStart)
                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesToCone3)
                // Cone4
                .setReversed(false)
                .splineTo(new Vector2d(FieldPoseConstants.LeftAutoConstants.stack.getX(), FieldPoseConstants.LeftAutoConstants.stack.getY()), FieldPoseConstants.LeftAutoConstants.stack.getHeading())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, gripper::gripCone)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, slides::slidesToMedium)
                .UNSTABLE_addTemporalMarkerOffset(0.11, arm::armToSCORE)
                .UNSTABLE_addTemporalMarkerOffset(0.11, wrist::toScoring)
                .setReversed(true)
                .splineTo(new Vector2d(FieldPoseConstants.LeftAutoConstants.closeMidJunction.getX(), FieldPoseConstants.LeftAutoConstants.closeMidJunction.getY()), FieldPoseConstants.LeftAutoConstants.closeMidJunction.getHeading())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, slides::slidesDrop)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, arm::armToDrop)
                .UNSTABLE_addTemporalMarkerOffset(-0.7, gripper::releaseCone)
                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::armToStart)
                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesToCone3)
                // Cone5
                .setReversed(false)
                .splineTo(new Vector2d(FieldPoseConstants.LeftAutoConstants.stack.getX(), FieldPoseConstants.LeftAutoConstants.stack.getY()), FieldPoseConstants.LeftAutoConstants.stack.getHeading())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, gripper::gripCone)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, slides::slidesToMedium)
                .UNSTABLE_addTemporalMarkerOffset(0.11, arm::armToSCORE)
                .UNSTABLE_addTemporalMarkerOffset(0.11, wrist::toScoring)
                .setReversed(true)
                .splineTo(new Vector2d(FieldPoseConstants.LeftAutoConstants.closeMidJunction.getX(), FieldPoseConstants.LeftAutoConstants.closeMidJunction.getY()), FieldPoseConstants.LeftAutoConstants.closeMidJunction.getHeading())
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, slides::slidesDrop)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, arm::armToDrop)
                .UNSTABLE_addTemporalMarkerOffset(-0.7, gripper::releaseCone)
                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::armToStart)
                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesToGround)
                .build();


//        TrajectorySequence High_Junction = drive.trajectorySequenceBuilder(RightAutoConstants.startPose)
//                .forward(55)
//                .setReversed(true)
//                .splineTo(new Vector2d(RightAutoConstants.closeMidJunction.getX(), RightAutoConstants.closeMidJunction.getY()), RightAutoConstants.closeMidJunction.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(0.1, intake::open)
//                .UNSTABLE_addTemporalMarkerOffset(-3, gripper::gripCone)
//                .UNSTABLE_addTemporalMarkerOffset(-1.75, slides::slidesToMedium)
//                .UNSTABLE_addTemporalMarkerOffset(-1, arm::armScoring)
//                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesDrop)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, gripper::releaseCone)
//                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesUp)
//                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToCone1)
//                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToStack)
//                .waitSeconds(0.5)
//                //Stack 1
//                .setReversed(false)
//                .splineTo(new Vector2d(RightAutoConstants.stack.getX(), RightAutoConstants.stack.getY()), RightAutoConstants.stack.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, gripper::gripCone)
//                .UNSTABLE_addTemporalMarkerOffset(0.01, slides::slidesToMedium)
//                .setReversed(true)
//                .splineTo(new Vector2d(RightAutoConstants.MidJunction.getX(), RightAutoConstants.MidJunction.getY()), RightAutoConstants.MidJunction.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.9, arm::armScoring)
//                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesDrop)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, gripper::releaseCone)
//                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesUp)
//                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToCone2)
//                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToGround)
//                .waitSeconds(0.25)
//                .setReversed(false)
//                //Stack 2
//                .splineTo(new Vector2d(RightAutoConstants.stack.getX()-1.75, RightAutoConstants.stack.getY()), RightAutoConstants.stack.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, gripper::gripCone)
//                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesToMedium)
//                .setReversed(true)
//                .splineTo(new Vector2d(RightAutoConstants.MidJunction.getX(), RightAutoConstants.MidJunction.getY()), RightAutoConstants.MidJunction.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.9, arm::armScoring)
//                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesDrop)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, gripper::releaseCone)
//                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesUp)
//                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToCone3)
//                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToGround)
//                .waitSeconds(0.25)
//                .setReversed(false)
//                //Stack 3
//                .splineTo(new Vector2d(RightAutoConstants.stack.getX()-3.5, RightAutoConstants.stack.getY()), RightAutoConstants.stack.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, gripper::gripCone)
//                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesToMedium)
//                .setReversed(true)
//                .splineTo(new Vector2d(RightAutoConstants.MidJunction.getX(), RightAutoConstants.MidJunction.getY()), RightAutoConstants.MidJunction.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.9, arm::armScoring)
//                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesDrop)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, gripper::releaseCone)
//                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesUp)
//                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToCone4)
//                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToGround)
//                .waitSeconds(0.25)
//                .setReversed(false)
//                //Stack 4
//                .splineTo(new Vector2d(RightAutoConstants.stack.getX()-5.5, RightAutoConstants.stack.getY()-0.5), RightAutoConstants.stack.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, gripper::gripCone)
//                .UNSTABLE_addTemporalMarkerOffset(0.01, slides::slidesToMedium)
//                .setReversed(true)
//                .splineTo(new Vector2d(RightAutoConstants.MidJunction.getX()-3.5, RightAutoConstants.MidJunction.getY()+0.5), RightAutoConstants.MidJunction.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.9, arm::armScoring)
//                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesDrop)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, gripper::releaseCone)
//                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesUp)
//                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToStart)
//                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToGround)
//                .waitSeconds(0.25)
//                .setReversed(false)
//                .splineTo(new Vector2d(RightAutoConstants.middleTile.getX(), RightAutoConstants.middleTile.getY()), RightAutoConstants.middleTile.getHeading())
//                .waitSeconds(0.5)
//                .build();

//        Trajectory trajLeft = drive.trajectoryBuilder(High_Junction.end())
//                .back(26)
//                .build();
//
//        Trajectory trajRight = drive.trajectoryBuilder(High_Junction.end())
//                .forward(22)
//                .build();

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

        mecanumDrive.followTrajectorySequence(testSequence);


//        if (tagOfInterest == null || tagOfInterest.id == Middle) {
//            // Do nothing
//
//        } else if (tagOfInterest.id == Left) {
//            // Left Code
//            drive.followTrajectory(trajLeft);
//
//
//        } else if (tagOfInterest.id == Right) {
//            // Right Code
//            drive.followTrajectory(trajRight);
//
//        } else {
//            // Do nothing
//        }

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        for (String s : Arrays.asList(String.format("\nDetected tag ID=%d", detection.id), String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER), String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER), String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER), String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)), String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)), String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)))) {
            telemetry.addLine(s);
        }
    }
}
