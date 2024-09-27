package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DelayForSeconds extends CommandBase {

    double m_duration;

    ElapsedTime time = new ElapsedTime();


    public DelayForSeconds(double duration)
    {
        m_duration = duration;
    }

    @Override
    public void initialize()
    {
        time.reset();
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return (time.seconds() > m_duration);
    }

}
