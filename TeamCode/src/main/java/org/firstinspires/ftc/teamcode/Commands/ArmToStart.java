package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

public class ArmToStart extends CommandBase {

    private Arm m_arm;
    private double m_duration;
    private ElapsedTime m_timer;


    public ArmToStart(Arm arm)
    {
        m_arm = arm;
        m_duration = 0.0;
        m_timer = new ElapsedTime();
    }

    public ArmToStart(Arm arm, double duration)
    {
        m_arm = arm;
        m_duration = duration;
        m_timer = new ElapsedTime();
    }

    @Override
    public void initialize()
    {
        m_timer.reset();
        m_arm.armToStart();
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        return m_timer.seconds() > m_duration;
    }

}
