package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

public class _TemplateCommand extends CommandBase {

    // Subsystems go here
    Subsystem m_s;

    /** Constructor for the command.
     * Takes in subsystems required to execute the command.
     * @param s the subsystem for this command (can be multiple)
     */
    public _TemplateCommand(Subsystem s) // TODO
    {

    }

    /** Execute startup commands.
     * Runs once when the command is scheduled.
     */
    @Override
    public void initialize()
    {

    }

    /** Execute continuous commands.
     *  Runs repeatedly, after initialization, once the command is scheduled.
     */
    @Override
    public void execute()
    {

    }

    /** Execute any cleanup commands.
     *  Runs once when the command ends.
     * @param interrupted whether this command was interrupted by another command.
     */
    @Override
    public void end(boolean interrupted)
    {

    }

    /** Determines the condition on which this command ends.
     * This determines whether a command finishes "gracefully," on its own,
     *      rather than whether it was interrupted.
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished()
    {
        return true; // TODO
    }

}
