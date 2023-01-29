package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_High;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1_Start;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_High;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2_Start;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Grab;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Release;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_High;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_Medium;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_Start;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Auto.Testing.PID;
import org.firstinspires.ftc.teamcode.BlakeStuff.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.EelverHardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "RRScoringStack")
public class RRStackScoring extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

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

    static final double FEET_PER_METER = 3.28084;

    EelverHardware hardware = new EelverHardware();


    @Override

    public void runOpMode() {

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

        hardware.init(hardwareMap);



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(37, 65, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory High_Junction = drive.trajectoryBuilder(startPose)
                .back(35)
                .splineTo(new Vector2d(35, 15), Math.toRadians(225))
                .build();

        Trajectory Back = drive.trajectoryBuilder(High_Junction.end())
                .back(9)
                .build();

        Trajectory Forward = drive.trajectoryBuilder(Back.end())
                .forward(13)
                .build();

        Pose2d forwardAfterTurn = new Pose2d(
                Forward.end().component1(),
                Forward.end().component2(),
                Math.toRadians(0));

        Trajectory Forward2 = drive.trajectoryBuilder(forwardAfterTurn)
                .forward(20)
                .build();

        Trajectory trajMiddle = drive.trajectoryBuilder(Forward2.end())
                .back(25)
                .build();

        Trajectory trajRight = drive.trajectoryBuilder(Forward2.end())
                .back(45)
                .build();




//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .splineTo(new Vector2d(35, 15), Math.toRadians(225))
//                .build();



        hardware.gripCone();
        hardware.armWiggle();

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


        hardware.setSlidesPower(1);
        hardware.slidesToHigh();

        drive.followTrajectory(High_Junction);

        while(opModeIsActive() && hardware.slidesAreBusy());

        hardware.armToHigh();

        drive.followTrajectory(Back);
        hardware.releaseCone();
        sleep(200);
        drive.followTrajectory(Forward);

        hardware.armToStart();
        sleep(500);
        // Returning
        hardware.setSlidesPower(0.8);
        hardware.slidesToStart();
        Gripper.setPosition(Gripper_Release);

        drive.turn(Math.toRadians(-45));
        drive.followTrajectory(Forward2);


        if (tagOfInterest == null || tagOfInterest.id == Left) {
            // Do nothing

        } else if (tagOfInterest.id == Middle) {
            // Middle Code
            drive.followTrajectory(trajMiddle);

        } else if (tagOfInterest.id == Right) {
            // Right Code
            drive.followTrajectory(trajRight);

        } else {
            // Do nothing
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
