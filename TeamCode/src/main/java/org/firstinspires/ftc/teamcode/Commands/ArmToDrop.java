package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

public class ArmToDrop extends CommandBase {

    private Arm m_arm;

    public ArmToDrop(Arm arm)
    {
        m_arm = arm;
    }

    @Override
    public void initialize()
    {
        m_arm.armToDrop();
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
