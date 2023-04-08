package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

public class ArmToScoring extends CommandBase {

    private Arm m_arm;

    public ArmToScoring(Arm arm)
    {
        m_arm = arm;
    }

    @Override
    public void initialize()
    {
        m_arm.armToSCORE();
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
