package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Gripper;

public class GripperReleaseCone extends CommandBase {

    Gripper m_gripper;


    public GripperReleaseCone(Gripper gripper)
    {
        m_gripper = gripper;
    }

    @Override
    public void initialize()
    {
        m_gripper.releaseCone();
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return true;
    }

}
