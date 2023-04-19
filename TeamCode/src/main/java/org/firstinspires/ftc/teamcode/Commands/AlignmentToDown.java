package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Alignment;

public class AlignmentToDown extends CommandBase {

    Alignment m_alignment;


    public AlignmentToDown(Alignment alignment)
    {
        m_alignment = alignment;
    }

    @Override
    public void initialize()
    {
        m_alignment.down();
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
