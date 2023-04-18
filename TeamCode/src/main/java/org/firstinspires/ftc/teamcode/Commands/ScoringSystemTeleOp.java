package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

public class ScoringSystemTeleOp extends CommandBase {

    private enum SlidesState {
        GROUND,
        LOW,
        MEDIUM,
        HIGH;

        public boolean scoring()
        {
            switch(this)
            {
                case GROUND: return false;
                default: return true;
            }
        }

        public boolean ground()
        {
            return this == GROUND;
        }

        public double getMoveTime()
        {
            switch(this) // FIXME correct delay values
            {
                case LOW:       return 0.0;
                case MEDIUM:    return 0.6;
                case HIGH:      return 1.0;
                default:        return -1.0;
            }
        }
    }

    // Subsystems
    private final Intake m_intake;
    private final Gripper m_gripper;
    private final Arm m_arm;
    private final Slides m_slides;

    private final Telemetry m_telemetry;
    private final Gamepad m_gamepad1;

    // Constants
    private final double INTAKE_TRIGGER_THRESHOLD = 0.1;

    // State variables
    private boolean gp1_lb_pressedLastIteration = false;
    private SlidesState slidesState = SlidesState.GROUND;


    public ScoringSystemTeleOp(Intake intake, Gripper gripper, Arm arm, Slides slides, Telemetry telemetry, Gamepad gamepad1) {
        m_intake    = intake;
        m_gripper   = gripper;
        m_arm       = arm;
        m_slides    = slides;

        m_telemetry = telemetry;
        m_gamepad1  = gamepad1;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(gripper, intake, arm, slides);
    }

    @Override
    public void initialize()
    {
        m_gripper.releaseCone();
    }

    @Override
    public void execute()
    {
        /* ------------ Intake ------------ */
        /* TODO: @Lohan @Elinora
         *  If we always close the intake when it is running
         *  and always open it when it is off, should we combine
         *  these function calls so we only ever call open,
         *  close, and off (e.g., call open() and close() in
         *  those functions themselves instead of out here)?
         *  For discussion.
         */
        if(m_gamepad1.right_trigger > INTAKE_TRIGGER_THRESHOLD) {
            m_intake.in();
            m_intake.close();
        } else if(m_gamepad1.right_bumper) {
            m_intake.out();
            m_intake.close();
        } else if (m_gripper.isGripping()) { // FIXME actions are same as "else" state
            m_intake.off();
            m_intake.open();
        } else {
            m_intake.off();
            m_intake.open();
        }

        /* ------------ Gripper ------------ */
        /* TODO: @Elinora @Lohan (https://docs.ftclib.org/ftclib/v/v2.0.0/features/gamepad-extensions)
         *  Looks like this toggling functionality might be already implemented by
         *  FTC Lib in their GamepadEx class. See wasJustPressed() function. We could
         *  consider using theirs, but nothing wrong with keeping ours.
         */
        boolean pressed = m_gamepad1.left_bumper;
        if (pressed & !gp1_lb_pressedLastIteration) m_gripper.toggle();
        gp1_lb_pressedLastIteration = pressed;

        /* ------------ Arm/Slides ------------ */

        /* TODO @Elinora @Lohan
         *  We need to see what happens when we hold/mash the buttons, as it may
         *  cause multiple conflicting command groups to be scheduled concurrently.
         *  We might be able to fix this issue by moving subsystem requirements
         *  into each command that directly uses them.
         */
        if(slidesState.scoring())
        {
            if(m_gamepad1.a) // return
            {
                schedule(new SequentialCommandGroup(
                        new ArmToStart(m_arm, calculateSlidesDelay()),
                        new SlidesToGround(m_slides)
                ));

                slidesState = SlidesState.GROUND;
            }
            else if(m_gamepad1.b) // scoring high
            {
                schedule(new SequentialCommandGroup(
                        new SlidesToHigh(m_slides) // ends when at a safe height for arm to move
                ));

                slidesState = SlidesState.HIGH;
            }
            else if(m_gamepad1.y) // scoring medium
            {
                schedule(new SequentialCommandGroup(
                        new SlidesToMedium(m_slides) // ends when at a safe height for arm to move
                ));

                slidesState = SlidesState.MEDIUM;
            }
            else if(m_gamepad1.x) // scoring low
            {
                schedule(new SequentialCommandGroup(
                        new SlidesToLow(m_slides) // ends when at a safe height for arm to move
                ));

                slidesState = SlidesState.LOW;
            }
        }
        else // slidesState.ground()
        {
            if(m_gamepad1.b) // scoring high
            {
                schedule(new SequentialCommandGroup(
                        new SlidesToHigh(m_slides), // ends when at a safe height for arm to move
                        new ArmToPreScore(m_arm)
                ));

                slidesState = SlidesState.HIGH;
            }
            else if(m_gamepad1.y) // scoring medium
            {
                schedule(new SequentialCommandGroup(
                        new SlidesToMedium(m_slides), // ends when at a safe height for arm to move
                        new ArmToPreScore(m_arm)
                ));

                slidesState = SlidesState.MEDIUM;
            }
            else if(m_gamepad1.x) // scoring low
            {
                schedule(new SequentialCommandGroup(
//                        new SlidesToLow(m_slides), // ends when at a safe height for arm to move
                        new ArmToPreScore(m_arm)
                ));

                slidesState = SlidesState.LOW;
            }
        }

        /* ------------ Telemetry ------------ */
        m_telemetry.addData("slides move time", slidesState.getMoveTime());
        m_telemetry.addData("slides delay", calculateSlidesDelay());
        m_telemetry.update();
    }

    @Override
    public boolean isFinished() { return false; }



    /**
     * Calculates the delay before we start returning slides to the start state.
     * @return the calculated delay, with minimum at least 0.0 seconds.
     */
    private double calculateSlidesDelay()
    {
        final double ARM_MOVE_TIME = 1.0; // FIXME correct this value
        double slidesMoveTime = slidesState.getMoveTime();

        return Math.max(ARM_MOVE_TIME - slidesMoveTime, 0.0);
    }


    /**
     * Schedules commands/command groups. Identical to CommandOpMode.schedule().
     * @param commands the command(s)/group(s) to be scheduled.
     */
    public void schedule(Command... commands)
    {
        CommandScheduler.getInstance().schedule(commands);
    }
}
