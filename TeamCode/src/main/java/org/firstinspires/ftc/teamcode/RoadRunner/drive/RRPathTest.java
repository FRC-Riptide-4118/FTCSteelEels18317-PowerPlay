package orgpackage org.firstinspires.ftc.teamcode.RoadRunner.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RRPathTest")
public class RRPathTest extends LinearOpMode {
    @Override

    public void runOpMode() {
        waitForStart();

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            // We want to start the bot at x: 10, y: -8, heading: 90 degrees
            Pose2d startPose = new Pose2d(-36, -36, Math.toRadians(90));

            drive.setPoseEstimate(startPose);

            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .forward(5)
                    .splineTo(new Vector2d(-31, -28), Math.toRadians(45))
                    .build();

            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                    .build();

            drive.followTrajectory(traj1);
        }
    }

