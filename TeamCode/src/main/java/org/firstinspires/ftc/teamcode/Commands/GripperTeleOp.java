package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Grab;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Gripper;

/**
 * An example command that uses an example subsystem.
 */
public class GripperTeleOp extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Gripper m_gripper;
    private final Gamepad m_gamepad1;
    private boolean pressedLastIteration = false;

    /**
     * Creates a new ExampleCommand.
     *
     * @param gripper The subsystem used by this command.
     */
    public GripperTeleOp(Gripper gripper, Gamepad gamepad1) {
        m_gripper    = gripper;
        m_gamepad1   = gamepad1;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(gripper);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        boolean pressed = m_gamepad1.left_bumper;
        if (pressed & !pressedLastIteration) {

            if(m_gripper.isGripping()) {
                m_gripper.releaseCone();
            }
            else {
                m_gripper.gripCone();
            }
        }
        pressedLastIteration = pressed;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
