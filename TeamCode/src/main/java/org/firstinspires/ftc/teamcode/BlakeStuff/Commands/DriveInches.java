package org.firstinspires.ftc.teamcode.BlakeStuff.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.BlakeStuff.Subsystems.Drivetrain;

public class DriveInches extends CommandBase
{
    // Subsystems used
    private final Drivetrain m_drivetrain;

    // Local variables
    private double inches;
    private double power;


    /**
     * Constructor to get the subsystem(s), parameters, and
     * require access to the subsystem(s) (if necessary).
     */
    public DriveInches(Drivetrain dt, double ins, double pwr)
    {
        m_drivetrain = dt;
        inches = ins;
        power = pwr;

        addRequirements(m_drivetrain);
    }

    /**
     * Do any setup required for the command (e.g., setting motor target positions).
     * This method is called once by the scheduler when an instance of this command
     * is scheduled.
     */
    @Override
    public void initialize()
    {
        // Set target positions
        // Set run modes
        // Set powers
    }

    /**
     * Do any recurring tasks for this command (e.g., updating motor powers).
     * This method is called repeatedly by the scheduler when an instance of
     * this command is scheduled.
     */
    @Override
    public void execute()
    {

    }

    /**
     * Performs any tasks needed to end the current command.
     */
    @Override
    public void end(boolean interrupted)
    {
        m_drivetrain.stop();
    }

    /**
     * Returns whether this command has finished, so the scheduler may stop it if done.
     * @return whether this command has finished
     */
    @Override
    public boolean isFinished()
    {
        return m_drivetrain.isBusy();
    }

}
