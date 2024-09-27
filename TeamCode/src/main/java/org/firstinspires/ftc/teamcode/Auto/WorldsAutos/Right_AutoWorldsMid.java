package org.firstinspires.ftc.teamcode.Auto.WorldsAutos;

import static org.firstinspires.ftc.teamcode.Util.FieldPoseConstants.RightAutoConstants.MidJunction;
import static org.firstinspires.ftc.teamcode.Util.FieldPoseConstants.RightAutoConstants.parkMiddle;
import static org.firstinspires.ftc.teamcode.Util.FieldPoseConstants.RightAutoConstants.stack;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.Util.FieldPoseConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous(group = "Worlds")
public class Right_AutoWorldsMid extends LinearOpMode {

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


        mecanumDrive.setPoseEstimate(FieldPoseConstants.RightAutoConstants.startPose);


        TrajectorySequence sequenceMiddle = mecanumDrive.trajectorySequenceBuilder(FieldPoseConstants.RightAutoConstants.startPose)
                .forward(55)
                .UNSTABLE_addTemporalMarkerOffset(-12, arm::armToMiddle)
                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesToMedium)
                .UNSTABLE_addTemporalMarkerOffset(0, wrist::toScoring)
                .UNSTABLE_addTemporalMarkerOffset(0.5, alignment::score)
                .UNSTABLE_addTemporalMarkerOffset(0, arm::armToPreScore)
                .UNSTABLE_addTemporalMarkerOffset(-2, intake::close)
                .UNSTABLE_addTemporalMarkerOffset(0, intake::open)
                .setReversed(true)
                .splineTo(new Vector2d(MidJunction.getX()+1, MidJunction.getY()), MidJunction.getHeading())
                // PreLoad
                                .UNSTABLE_addTemporalMarkerOffset(0.0, arm::armToScore)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, gripper::releaseCone)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::toStack)
                                .UNSTABLE_addTemporalMarkerOffset(0.7, slides::slidesToCone1)
                // Cone 1
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(stack.getX()+23, stack.getY(), stack.getHeading()))
                .forward(inchesToStack)
                                .UNSTABLE_addTemporalMarkerOffset(0.0, gripper::gripCone)
                                .UNSTABLE_addTemporalMarkerOffset(0.0, alignment::down)
                                .UNSTABLE_addTemporalMarkerOffset(0.02, intake::out)
                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, slides::slidesToMedium)
                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, arm::armToPreScore)
                                .waitSeconds(wristIntakeArmDelay)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, wrist::toScoring)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, intake::off)
                                .UNSTABLE_addTemporalMarkerOffset(1.2, alignment::score)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(MidJunction.getX(), MidJunction.getY(), MidJunction.getHeading() + Math.PI))
//
                                .UNSTABLE_addTemporalMarkerOffset(0.0, arm::armToScore)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, gripper::releaseCone)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::toStack)
                                .UNSTABLE_addTemporalMarkerOffset(0.7, slides::slidesToCone2)
                // Cone 2
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(stack.getX()+23, stack.getY(), stack.getHeading()))
                .forward(inchesToStack)
                                .UNSTABLE_addTemporalMarkerOffset(0.0, gripper::gripCone)
                                .UNSTABLE_addTemporalMarkerOffset(0.0, alignment::down)
                                .UNSTABLE_addTemporalMarkerOffset(0.02, intake::out)
                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, slides::slidesToMedium)
                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, arm::armToPreScore)
                                .waitSeconds(wristIntakeArmDelay)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, wrist::toScoring)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, intake::off)
                                .UNSTABLE_addTemporalMarkerOffset(1.2, alignment::score)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(MidJunction.getX(), MidJunction.getY(), MidJunction.getHeading() + Math.PI))
                                .UNSTABLE_addTemporalMarkerOffset(0.0, arm::armToScore)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, gripper::releaseCone)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::toStack)
                                .UNSTABLE_addTemporalMarkerOffset(0.7, slides::slidesToCone3)
                // Cone 3
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(stack.getX()+23, stack.getY(), stack.getHeading()))
                .forward(inchesToStack)
                                .UNSTABLE_addTemporalMarkerOffset(0.0, gripper::gripCone)
                                .UNSTABLE_addTemporalMarkerOffset(0.0, alignment::down)
                                .UNSTABLE_addTemporalMarkerOffset(0.02, intake::out)
                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, slides::slidesToMedium)
                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, arm::armToPreScore)
                                .waitSeconds(wristIntakeArmDelay)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, wrist::toScoring)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, intake::off)
                                .UNSTABLE_addTemporalMarkerOffset(1.2, alignment::score)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(MidJunction.getX(), MidJunction.getY(), MidJunction.getHeading() + Math.PI))
                                .UNSTABLE_addTemporalMarkerOffset(0.0, arm::armToScore)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, gripper::releaseCone)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::toStack)
                                .UNSTABLE_addTemporalMarkerOffset(0.7, slides::slidesToCone4)
                // Cone 4
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(stack.getX()+23, stack.getY(), stack.getHeading()))
                .forward(inchesToStack)
                                .UNSTABLE_addTemporalMarkerOffset(0.0, gripper::gripCone)
                                .UNSTABLE_addTemporalMarkerOffset(0.0, alignment::down)
                                .UNSTABLE_addTemporalMarkerOffset(0.02, intake::out)
                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, slides::slidesToMedium)
                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, arm::armToPreScore)
                                .waitSeconds(wristIntakeArmDelay)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, wrist::toScoring)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, intake::off)
                                .UNSTABLE_addTemporalMarkerOffset(1.2, alignment::score)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(MidJunction.getX(), MidJunction.getY(), MidJunction.getHeading() + Math.PI))
                                .UNSTABLE_addTemporalMarkerOffset(0.0, arm::armToScore)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, gripper::releaseCone)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::toStart)
                                .UNSTABLE_addTemporalMarkerOffset(0.7, slides::slidesToCone5)
                // Cone 5??
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(stack.getX()+23, stack.getY(), stack.getHeading()))
                .forward(inchesToStack)
                                .UNSTABLE_addTemporalMarkerOffset(0.0, gripper::gripCone)
                                .UNSTABLE_addTemporalMarkerOffset(0.0, alignment::down)
                                .UNSTABLE_addTemporalMarkerOffset(-0.1, intake::in)
                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, slides::slidesToMedium)
                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, arm::armToPreScore)
                                .waitSeconds(wristIntakeArmDelay)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, wrist::toScoring)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, intake::off)
                                .UNSTABLE_addTemporalMarkerOffset(1.2, alignment::score)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(MidJunction.getX(), MidJunction.getY(), MidJunction.getHeading() + Math.PI))
                                .UNSTABLE_addTemporalMarkerOffset(0.0, arm::armToScore)
                                .UNSTABLE_addTemporalMarkerOffset(0.2, gripper::releaseCone)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
                                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::toStack)
                                .UNSTABLE_addTemporalMarkerOffset(0.7, slides::slidesToCone5)
                                .UNSTABLE_addTemporalMarkerOffset(0.9, alignment::up)
                // PARK PARK PARK
                .setReversed(false)
                .lineToLinearHeading(parkMiddle)
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

        if (tagOfInterest == null || tagOfInterest.id == Middle) {
            mecanumDrive.followTrajectorySequence(sequenceMiddle);

        } else if (tagOfInterest.id == Left) {
//            mecanumDrive.followTrajectorySequence(sequenceLeft);

        } else if (tagOfInterest.id == Right) {
//            mecanumDrive.followTrajectorySequence(sequenceRight);

        } else {
            mecanumDrive.followTrajectorySequence(sequenceMiddle);
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
