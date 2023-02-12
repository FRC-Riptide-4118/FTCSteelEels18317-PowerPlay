package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeServos;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;


/**
 * An example command that uses an example subsystem.
 */
public class IntakeServosTeleOp extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Gripper m_gripper;
    private final IntakeServos m_intakeServos;

    /**
     * Creates a new ExampleCommand.
     *
     * @param intakeServos The subsystem used by this command.
     * @param gripper The subsystem used by this command.
     */
    public IntakeServosTeleOp(IntakeServos intakeServos, Gripper gripper, Gamepad gamepad1) {
        m_gripper         = gripper;
        m_intakeServos    = intakeServos;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intakeServos, gripper);
    }

    @Override
    public void initialize() {
        m_intakeServos.intakeServoOut();
    }

    @Override
    public void execute() {
        if (m_gripper.isGripping() || m_gripper.Distance() == true) {
            m_intakeServos.intakeServoOut();
        }
        else {
            m_intakeServos.intakeServoIn();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
