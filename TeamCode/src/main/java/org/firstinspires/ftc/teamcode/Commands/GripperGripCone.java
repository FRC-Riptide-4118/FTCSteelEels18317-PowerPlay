package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Gripper;

public class GripperGripCone extends CommandBase {

    Gripper m_gripper;


    public GripperGripCone(Gripper gripper)
    {
        m_gripper = gripper;
    }

    @Override
    public void initialize()
    {
        m_gripper.gripCone();
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
