package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.DrivetrainTeleOp;
import org.firstinspires.ftc.teamcode.Commands.ScoringSystemTeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@Disabled
@TeleOp(group = "Testing")
public class TeleOpCommandBased extends CommandOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize()
    {
        // Telemetry
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());


        // Subsystems
        Drivetrain drivetrain   = new Drivetrain(hardwareMap);
        Intake intake           = new Intake(hardwareMap);
        Gripper gripper         = new Gripper(hardwareMap);
        Arm arm                 = new Arm(hardwareMap);
        Slides slides           = new Slides(hardwareMap);


        // Schedule commands
        schedule(
                new DrivetrainTeleOp(drivetrain, gamepad1),
                new ScoringSystemTeleOp(intake, gripper, arm, slides, telemetry, gamepad1));
    }
}
