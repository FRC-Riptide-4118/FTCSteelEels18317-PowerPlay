package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Subsystems.Gripper;

public class GripperRelease extends CommandBase {

    Gripper m_gripper;


    public GripperRelease(Gripper gripper)
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
