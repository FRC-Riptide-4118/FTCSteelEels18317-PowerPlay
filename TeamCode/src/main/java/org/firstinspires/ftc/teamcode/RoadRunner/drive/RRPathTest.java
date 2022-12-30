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

        Trajectory testing = drivetrain.trajectoryBuilder(new Pose2d(4.4,-0.4, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(10.8,1.2, Math.toRadians(18)), Math.toRadians(18))
                .splineToSplineHeading(new Pose2d(15.6,3.2, Math.toRadians(-8)), Math.toRadians(-8))
                .splineToSplineHeading(new Pose2d(21.6,2.4, Math.toRadians(-30)), Math.toRadians(-30))
                .splineToSplineHeading(new Pose2d(26.8,-0.4, Math.toRadians(-67)), Math.toRadians(-67))
                .splineToSplineHeading(new Pose2d(28.4,-4.4, Math.toRadians(-86)), Math.toRadians(-86))
                .splineToSplineHeading(new Pose2d(28.8,-9.6, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(29.2,-22.4, Math.toRadians(-81)), Math.toRadians(-81))
                .build();

        drivetrain.followTrajectory(testing);
    }
}
