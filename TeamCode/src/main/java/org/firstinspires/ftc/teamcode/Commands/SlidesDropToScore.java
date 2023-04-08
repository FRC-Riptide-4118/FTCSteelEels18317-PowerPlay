package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Slides;

public class SlidesDropToScore extends CommandBase {

    private Slides m_slides;

    public SlidesDropToScore(Slides slides)
    {
        m_slides = slides;
    }

    @Override
    public void initialize()
    {
        m_slides.slidesDrop();
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return m_slides.atSafeHeight();
    }

}
