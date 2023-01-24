package org.firstinspires.ftc.teamcode.BlakeStuff.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.BlakeStuff.Subsystems.Gripper;

public class GripCone extends CommandBase
{
    // Subsystems
    private final Gripper m_gripper;

    /**
     * Constructor.
     */
    public GripCone(Gripper grip)
    {
        m_gripper = grip;

        addRequirements(m_gripper);
    }

    @Override
    public void initialize()
    {
        m_gripper.grip();
    }

    @Override
    public boolean isFinished()
    {
        return true;
        // ^-- Servos "finish" immediately, so we just return true.
        // We could alternatively start a timer and check it here
        // against the expected time to move the servo.
    }
}
