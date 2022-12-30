package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RRPathTest")
public class RRPathTest extends LinearOpMode {
    @Override

    public void runOpMode() {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory testing = drivetrain.trajectoryBuilder(new Pose2d(0.4,0, Math.toRadians(-3)), Math.toRadians(-3))
                .splineToSplineHeading(new Pose2d(48,0.4, Math.toRadians(-2)), Math.toRadians(-2))
                .splineToSplineHeading(new Pose2d(48,-35.6, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(41.2,-43.2, Math.toRadians(-90)), Math.toRadians(-90))
                .build();
        drivetrain.followTrajectory(testing);
    }
}
