package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
/**
 * An example command that uses an example subsystem.
 */
public class DrivetrainTeleOp extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain m_drivetrain;
    private final Gamepad m_gamepad1;
    private boolean pressedLastIteration = false;

    /**
     * Creates a new ExampleCommand.
     *
     * @param drivetrain The subsystem used by this command.
     */
    public DrivetrainTeleOp(Drivetrain drivetrain, Gamepad gamepad1) {
        m_drivetrain    = drivetrain;
        m_gamepad1   = gamepad1;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Gamepad controls
        double drive        = -m_gamepad1.left_stick_y;
        double strafe       = m_gamepad1.left_stick_x;
        double twist        = m_gamepad1.right_stick_x;
        double slowMode     = Math.abs(1.25 - m_gamepad1.left_trigger);

        m_drivetrain.setMecanumPower(drive, strafe, twist, slowMode);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
