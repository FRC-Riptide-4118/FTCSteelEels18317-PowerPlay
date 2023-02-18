package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Commands.GripperTeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeServos;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

public class CommandOpModeTesting extends CommandOpMode {
    // in your implementation of CommandOpMode
//    private Arm                 m_arm;
//    private Slides              m_slides;
//    private Intake              m_intake;
//    private IntakeServos        m_intakeServos;
    private Gripper             m_gripper;
//    private Drivetrain          m_drivetrain;

    @Override
    public void initialize() {
        // initialize hardware
        m_gripper       = new Gripper(hardwareMap);

        // schedule all commands
        schedule(new GripperTeleOp(m_gripper, gamepad1));
    }
}
