package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

/**
 * An example command that uses an example subsystem.
 */
public class ArmTeleOp extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Arm          m_arm;
    private final Gamepad m_gamepad1;

    /**
     * Creates a new ExampleCommand.
     *
     * @param arm The subsystem used by this command.
     */
    public ArmTeleOp(Arm arm, Gamepad gamepad1) {
        m_arm        = arm;
        m_gamepad1   = gamepad1;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.armToStart();
    }

    @Override
    public void execute() {
        // Stack Scoring
        if(m_gamepad1.dpad_left) {
            m_arm.armToCone1();
        }
        if(m_gamepad1.dpad_right) {
            m_arm.armToStart();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
