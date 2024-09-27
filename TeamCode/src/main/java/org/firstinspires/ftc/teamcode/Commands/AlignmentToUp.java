package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Alignment;

public class AlignmentToUp extends CommandBase {

    Alignment m_alignment;


    public AlignmentToUp(Alignment alignment)
    {
        m_alignment = alignment;
    }

    @Override
    public void initialize()
    {
        m_alignment.up();
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
