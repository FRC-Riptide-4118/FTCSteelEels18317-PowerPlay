package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Subsystems.Alignment;

public class AlignmentToScoring extends CommandBase {

    Alignment m_alignment;


    public AlignmentToScoring(Alignment alignment)
    {
        m_alignment = alignment;
    }

    @Override
    public void initialize()
    {
        m_alignment.score();
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return true; // TODO
    }

}
