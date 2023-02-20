package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

/**
 * An example command that uses an example subsystem.
 */
public class SlidesTeleOp extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Slides     m_slides;
    private final Gamepad m_gamepad1;

    /**
     * Creates a new ExampleCommand.
     *
     * @param slides The subsystem used by this command.
     */
    public SlidesTeleOp(Slides slides, Gamepad gamepad1) {
        m_slides     = slides;
        m_gamepad1   = gamepad1;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(slides);
    }

    @Override
    public void initialize() {
        m_slides.slidesToStart();
    }

    @Override
    public void execute() {
        // Fine Control the Slides
        if(m_gamepad1.dpad_up) {
             m_slides.leftSlide.setTargetPosition(m_slides.leftSlide.getCurrentPosition() + 50);
             m_slides.rightSlide.setTargetPosition(m_slides.rightSlide.getCurrentPosition() + 50);
        }
        if(m_gamepad1.dpad_down) {
            m_slides.leftSlide.setTargetPosition(m_slides.leftSlide.getCurrentPosition() - 50);
            m_slides.rightSlide.setTargetPosition(m_slides.rightSlide.getCurrentPosition() - 50);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
