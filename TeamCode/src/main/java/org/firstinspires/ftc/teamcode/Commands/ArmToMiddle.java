package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;

public class ArmToMiddle extends CommandBase {

    private Arm m_arm;


    public ArmToMiddle(Arm arm)
    {
        m_arm = arm;
    }

    public ArmToMiddle(Arm arm, double duration)
    {
        m_arm = arm;
    }

    @Override
    public void initialize()
    {
        m_arm.armToMiddle();
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
