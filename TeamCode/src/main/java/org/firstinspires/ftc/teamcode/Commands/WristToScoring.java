package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class WristToScoring extends CommandBase {

    private Wrist m_wrist;

    public WristToScoring(Wrist wrist)
    {
        m_wrist = wrist;
    }

    @Override
    public void initialize()
    {
        m_wrist.toScoring();
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
