package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

public class ArmToStart extends CommandBase {

    private Arm m_arm;


    public ArmToStart(Arm arm)
    {
        m_arm = arm;
    }

    public ArmToStart(Arm arm, double duration)
    {
        m_arm = arm;
    }

    @Override
    public void initialize()
    {
        m_arm.toStart();
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
