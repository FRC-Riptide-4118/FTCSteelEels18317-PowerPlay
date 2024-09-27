package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class WristToStart extends CommandBase {

    private Wrist m_wrist;

    public WristToStart(Wrist wrist)
    {
        m_wrist = wrist;
    }

    @Override
    public void initialize()
    {
        m_wrist.toStart();
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
