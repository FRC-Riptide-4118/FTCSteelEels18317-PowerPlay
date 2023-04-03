package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.ScoringSystemTeleOp;
import org.firstinspires.ftc.teamcode.Commands.TestScoringSystemTeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Alignment;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

@TeleOp(group = "Testing")
public class TeleOpNewArm extends CommandOpMode {

    private Telemetry m_telemetry;

    @Override
    public void initialize()
    {
        m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Gripper     gripper     = new Gripper(hardwareMap);
        Arm         arm         = new Arm(hardwareMap);
        Wrist       wrist       = new Wrist(hardwareMap);
        Intake      intake      = new Intake(hardwareMap);
        Alignment   alignment   = new Alignment(hardwareMap);

        schedule(new TestScoringSystemTeleOp(gripper, arm, wrist, intake, alignment,
                telemetry, gamepad1));
    }
}
