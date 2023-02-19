package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Commands.ArmTeleOp;
import org.firstinspires.ftc.teamcode.Commands.DrivetrainTeleOp;
import org.firstinspires.ftc.teamcode.Commands.GripperTeleOp;
import org.firstinspires.ftc.teamcode.Commands.IntakeServosTeleOp;
import org.firstinspires.ftc.teamcode.Commands.IntakeTeleOp;
import org.firstinspires.ftc.teamcode.Commands.ScoringTeleOp;
import org.firstinspires.ftc.teamcode.Commands.SlidesTeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeServos;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

public class CommandOpModeTesting extends CommandOpMode {
    // in your implementation of CommandOpMode
    private Arm                 m_arm;
    private Slides              m_slides;
    private Intake              m_intake;
    private IntakeServos        m_intakeServos;
    private Gripper             m_gripper;
    private Drivetrain          m_drivetrain;

    @Override
    public void initialize() {
        // initialize hardware
        m_gripper       = new Gripper(hardwareMap);
//        m_drivetrain    = new Drivetrain(hardwareMap);
//        m_slides        = new Slides(hardwareMap);
//        m_arm           = new Arm(hardwareMap);
//        m_intake        = new Intake(hardwareMap);
//        m_intakeServos  = new IntakeServos(hardwareMap);

        // schedule all commands
        schedule(
                new GripperTeleOp(m_gripper, gamepad1)/*,
                new DrivetrainTeleOp(m_drivetrain, gamepad1),
                new SlidesTeleOp(m_slides, gamepad1),
                new ArmTeleOp(m_arm, gamepad1),
                new IntakeTeleOp(m_intake, m_gripper, gamepad1),
                new IntakeServosTeleOp(m_intakeServos, m_gripper, gamepad1),
                new ScoringTeleOp(m_arm, m_slides, gamepad1)*/);
    }
}
