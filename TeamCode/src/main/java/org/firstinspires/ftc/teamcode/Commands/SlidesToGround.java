package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Slides;

public class SlidesToGround extends CommandBase {

    private Slides m_slides;


    public SlidesToGround(Slides slides)
    {
        m_slides = slides;
    }

    @Override
    public void initialize()
    {
        m_slides.slidesToGround();
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
