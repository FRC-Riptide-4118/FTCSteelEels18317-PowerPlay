package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Slides;

public class SlidesToMedium extends CommandBase {

    private Slides m_slides;

    public SlidesToMedium(Slides slides)
    {
        m_slides = slides;
    }

    @Override
    public void initialize()
    {
        m_slides.slidesToMedium();
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
