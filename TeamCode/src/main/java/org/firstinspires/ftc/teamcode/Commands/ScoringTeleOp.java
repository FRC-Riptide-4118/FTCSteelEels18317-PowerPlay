package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

/**
 * An example command that uses an example subsystem.
 */
public class ScoringTeleOp extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Arm        m_arm;
    private final Gamepad    m_gamepad1;
    private final Slides     m_slides;

    private boolean raisingToLow    = false;
    private boolean returning       = false;
    private boolean raisingToMiddle = false;
    private boolean raisingToHigh   = false;

    private ElapsedTime armInTimer;

    /**
     * Creates a new ExampleCommand.
     *  @param arm    The subsystem used by this command.
     *  @param slides The subsystem used by this command.
     */
    public ScoringTeleOp(Arm arm, Slides slides, Gamepad gamepad1) {
        m_arm        = arm;
        m_gamepad1   = gamepad1;
        m_slides     = slides;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm, slides);
        armInTimer = new ElapsedTime();
        armInTimer.reset();
    }

    @Override
    public void initialize() {
            m_arm.armToStart();
            m_slides.slidesToStart();
    }

    @Override
    public void execute() {
        /*-------Lift & Arm-------*/
        // Ground
        if(m_gamepad1.a) {
            returning = true;
            m_arm.armToStart();
            if (armInTimer.seconds() > 1.0) armInTimer.reset();
        }

        if(returning) {
            if(armInTimer.seconds() > 1.0) {
                m_slides.setSlidesPower(0.7);
                m_slides.slidesToStart();
                m_arm.armToStart();
                m_arm.armToStart();
                returning = false;
            }
        }

        // Low
        if(m_gamepad1.x) {
            raisingToLow = true;
            m_slides.setSlidesPower(1);
            m_slides.slidesToLow();
        }
        if(raisingToLow) {
            if(m_slides.leftSlide.getCurrentPosition() > 600) {
                m_arm.armScoring();
                raisingToLow = false;
            }
        }

        // Medium
        if(m_gamepad1.y) {
            raisingToMiddle = true;
            m_slides.setSlidesPower(1);
            m_slides.slidesToMedium();
        }
        if(raisingToMiddle) {
            if(m_slides.leftSlide.getCurrentPosition() > 600) {
                m_arm.armScoring();
                raisingToMiddle = false;
            }
        }

        // High
        if(m_gamepad1.b) {
            raisingToHigh = true;
            m_slides.setSlidesPower(1);
            m_slides.slidesToHigh();
        }
        if(raisingToHigh) {
            if(m_slides.leftSlide.getCurrentPosition() > 600) {
                m_arm.armScoring();
                raisingToHigh = false;
            }
        }
    }

   @Override
   public boolean isFinished() {
       return (
               (m_slides.leftSlide.getCurrentPosition() > 1950)
       );
   }

}
