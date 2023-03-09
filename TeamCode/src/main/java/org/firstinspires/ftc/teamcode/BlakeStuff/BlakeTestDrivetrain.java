package org.firstinspires.ftc.teamcode.BlakeStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Disabled
@Config
@Autonomous(name = "(Blake) Test Drivetrain", group = "Blake")
public class BlakeTestDrivetrain extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    SampleMecanumDrive drive;
    Pose2d startPose = new Pose2d(-37, 63, Math.toRadians(90));


    public void runOpMode()
    {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        waitForStart();

        while(opModeIsActive())
        {
            Pose2d driveInputs = new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );

            drive.setWeightedDrivePower(driveInputs);

            drive.update();

            Pose2d driveVelocity = drive.getPoseVelocity();
            Pose2d drivePos = drive.getPoseEstimate();
            dashboardTelemetry.addData("drive velocity x", driveVelocity.getX());
            dashboardTelemetry.addData("drive velocity y", driveVelocity.getY());
            dashboardTelemetry.addData("drive velocity heading", driveVelocity.getHeading());
            dashboardTelemetry.addData("test", drivePos);
            dashboardTelemetry.update();
        }
    }
}
