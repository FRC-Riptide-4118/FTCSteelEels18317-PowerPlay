package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

/**
 * An example command that uses an example subsystem.
 */
public class IntakeTeleOp extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;
    private final Gripper m_gripper;
    private final Gamepad m_gamepad1;


    /**
     * Creates a new ExampleCommand.
     *
     * @param intake The subsystem used by this command.
     * @param gripper The subsystem used by this command.
     */
    public IntakeTeleOp(Intake intake, Gripper gripper, Gamepad gamepad1) {
        m_gripper    = gripper;
        m_intake     = intake;
        m_gamepad1   = gamepad1;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake, gripper);
    }

    @Override
    public void initialize() {
        m_intake.intake(0);
    }

    @Override
    public void execute() {
        m_intake.intake(m_gamepad1.right_trigger);
        if(m_gamepad1.right_bumper) {
            m_intake.intakeOut();
        }
        else {
            m_intake.intake(0);
        }
    }

    @Override
    public boolean isFinished() {
        return m_gripper.isGripping();
    }
}
