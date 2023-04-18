package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Alignment;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class TestScoringSystemTeleOp extends CommandBase {

    private enum ScoringState
    {
        GROUND,
        READY,
        SCORING;

        @Override
        public String toString()
        {
            switch(this)
            {
                case GROUND: return "Ground";
                case READY: return "Ready";
                case SCORING: return "Scoring";
                default: return "!! unknown";
            }
        }

        public ScoringState get()
        {
            switch(this)
            {
                case GROUND : return ScoringState.GROUND;
                case READY  : return ScoringState.READY;
                case SCORING: return ScoringState.SCORING;
                default     : return null;
            }
        }
    }
    private enum IntakeState
    {
        IN,
        OUT,
        OFF;

        public IntakeState get()
        {
            switch(this)
            {
                case IN : return IN;
                case OUT: return OUT;
                case OFF: return OFF;
                default : return null;
            }
        }

    }


    // Subsystems
    private final Gripper m_gripper;
    private final Arm m_arm;
    private final Wrist m_wrist;
    private final Intake m_intake;
    private final Alignment m_alignment;
    private final Slides m_slides;

    private final Telemetry m_telemetry;
    private final Gamepad m_gamepad1;

    // State variables
    private ScoringState scoringState = ScoringState.GROUND;
    private IntakeState intakeState = IntakeState.OFF;

    // Note shortening pressedLastIteration to PLI
    private boolean gp1_lb_PLI = false;
    private boolean gp1_a_PLI = false;
    private boolean gp1_rt_PLI = false;
    private boolean gp1_rb_PLI = false;
    private boolean gp1_lsb_PLI = false;

    // Constants
    private final double TRIGGER_THRESHOLD = 0.5;
    private final double WRIST_SCORE_DELAY = 0.8;


    public TestScoringSystemTeleOp(Gripper gripper, Arm arm, Wrist wrist, Intake intake, Alignment alignment, Slides slides,
                                   Telemetry telemetry, Gamepad gamepad1) {
        m_gripper   = gripper;
        m_arm       = arm;
        m_wrist     = wrist;
        m_intake    = intake;
        m_alignment = alignment;
        m_slides    = slides;

        m_telemetry = telemetry;
        m_gamepad1  = gamepad1;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(gripper, arm, wrist, intake, alignment, slides);
    }

    @Override
    public void initialize()
    {
        m_gripper.releaseCone();
    }

    @Override
    public void execute()
    {
        /* Controls breakdown:
         *  - Intake State
         *      - press right trigger to toggle intaking mode (close intake, intake in, gripper should already be open)
         *          - when right trigger to toggle off (open intake, intake stop)
         *          - when cone detected (stop intake, open intake, grip cone)
         *      - press right bumper to toggle outtaking mode (close intake, intake out)
         *          - when toggle off outtaking mode (open intake, intake stop)
         *  - Ground State
         *      - (command order: alignment, slides, arm, delay, wrist)
         *      - x for low --> ready
         *      - y for medium --> ready
         *      - b for high --> ready
         *  - Ready state
         *      - x for low --> ready
         *      - y for medium --> ready
         *      - b for high --> ready
         *      - a for drop slides --> scoring
         *  - Scoring state
         *      - x for low --> ready
         *      - y for medium --> ready
         *      - b for high --> ready
         *      - a for return --> ground (align down, gripper open, delay then reset wrist, arm, slides)
         */


        ScoringState nextScoringState = scoringState.get();
        IntakeState nextIntakeState = intakeState.get();

        boolean lb_pressed = m_gamepad1.left_bumper;
        boolean a_pressed = m_gamepad1.a;
        boolean rt_pressed = m_gamepad1.right_trigger > TRIGGER_THRESHOLD;
        boolean rb_pressed = m_gamepad1.right_bumper;

        boolean lsb_pressed = m_gamepad1.left_stick_button;

        /* ------------ Intake ------------ */
        if(intakeState == IntakeState.OFF)
        {
            m_intake.open();
            m_intake.off();

            if(rt_pressed && !gp1_rt_PLI) {
                nextIntakeState = IntakeState.IN;
                if(scoringState == ScoringState.GROUND) m_arm.armToStart();
            }
            else if(rb_pressed && !gp1_rb_PLI) nextIntakeState = IntakeState.OUT;
            // else stay in OFF
        }
        else if(intakeState == IntakeState.IN)
        {
            m_intake.close();
            m_intake.in();

            if(m_gripper.coneDetected())
            {
                nextIntakeState = IntakeState.OFF;
                m_gripper.gripCone();
                m_arm.armToMiddle();
            }
            else
            {
                if(rt_pressed && !gp1_rt_PLI) nextIntakeState = IntakeState.OFF;
                else if(rb_pressed && !gp1_rb_PLI) nextIntakeState = IntakeState.OUT;
            }
        }
        else // (intakeState == IntakeState.OUT)
        {
            m_intake.close();
            m_intake.out();

            if(rt_pressed && !gp1_rt_PLI) nextIntakeState = IntakeState.IN;
            else if(rb_pressed && !gp1_rb_PLI) nextIntakeState = IntakeState.OFF;
        }

        /* ------------ Gripper ------------ */
        if (lb_pressed && !gp1_lb_PLI) m_gripper.toggle();

        /* ------------ Arm/Slides/Wrist ------------ */
        if(scoringState == ScoringState.GROUND)
        {
            if(m_gamepad1.left_stick_button && gp1_lsb_PLI)
            {
                m_arm.toggleConeFlip();
            }

            if(m_gamepad1.x)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new SlidesToLow(m_slides),
                        new ArmToPreScore(m_arm),
                        new DelayForSeconds(WRIST_SCORE_DELAY),
                        new WristToScoring(m_wrist)
                ));

                nextScoringState = ScoringState.READY;
            }

            if(m_gamepad1.y)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new SlidesToMedium(m_slides),
                        new DelayForSeconds(WRIST_SCORE_DELAY),
                        new WristToScoring(m_wrist)
                ));

                nextScoringState = ScoringState.READY;
            }

            if(m_gamepad1.b)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new SlidesToHigh(m_slides),
                        new DelayForSeconds(WRIST_SCORE_DELAY),
                        new WristToScoring(m_wrist)
                ));

                nextScoringState = ScoringState.READY;
            }

            if(m_gamepad1.a)
            {
                m_arm.armToStart();
                m_gripper.releaseCone();
            }
        }
        else if(scoringState == ScoringState.READY)
        {
            if(m_gamepad1.x)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new SlidesToLow(m_slides),
                        new DelayForSeconds(0.04),
                        new WristToScoring(m_wrist)
                ));

                nextScoringState = ScoringState.READY;
            }

            if(m_gamepad1.y)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new SlidesToMedium(m_slides),
                        new DelayForSeconds(0.04),
                        new WristToScoring(m_wrist)
                ));

                nextScoringState = ScoringState.READY;
            }


            if(m_gamepad1.b)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new SlidesToHigh(m_slides),
                        new DelayForSeconds(0.04),
                        new WristToScoring(m_wrist)
                ));

                nextScoringState = ScoringState.READY;
            }



            if(a_pressed && !gp1_a_PLI)
            {
                schedule(new SequentialCommandGroup(
//                        new SlidesDropToScore(m_slides),
                        new ArmToScore(m_arm)
                ));

                nextScoringState = ScoringState.SCORING;
            }
        }
        else // (ScoringState.SCORING)
        {
            if(m_gamepad1.x)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new SlidesToLow(m_slides),
                        new DelayForSeconds(0.04),
                        new WristToScoring(m_wrist)
                ));

                nextScoringState = ScoringState.READY;
            }

            if(m_gamepad1.y)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new SlidesToMedium(m_slides),
                        new DelayForSeconds(0.04),
                        new WristToScoring(m_wrist)
                ));

                nextScoringState = ScoringState.READY;
            }


            if(m_gamepad1.b)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new SlidesToHigh(m_slides),
                        new DelayForSeconds(0.04),
                        new WristToScoring(m_wrist)
                ));

                nextScoringState = ScoringState.READY;
            }

            if(a_pressed && !gp1_a_PLI)
            {
                m_gripper.releaseCone();

                schedule(new SequentialCommandGroup(
                        new AlignmentToDown(m_alignment),
                        new DelayForSeconds(0.5),
                        new ArmToStart(m_arm),
                        new SlidesToStart(m_slides),
                        new DelayForSeconds(0.2),
                        new WristToStart(m_wrist),
                        new DelayForSeconds(0.8),
                        new AlignmentToUp(m_alignment)
                ));

                nextScoringState = ScoringState.GROUND;
            }
        }


        // Update state
        scoringState = nextScoringState.get();
        intakeState = nextIntakeState.get();

        gp1_rb_PLI = rb_pressed;
        gp1_a_PLI = a_pressed;
        gp1_rt_PLI = rt_pressed;
        gp1_lb_PLI = lb_pressed;
        gp1_lsb_PLI = lsb_pressed;

        /* ------------ Telemetry ------------ */
        m_telemetry.addData("scoring state", scoringState.toString());
        m_telemetry.addData("intake state", intakeState.toString());
        m_telemetry.addData("gripping", m_gripper.getPosition());
        m_telemetry.addLine();
        m_telemetry.addData("wrist pos", m_wrist.getPosition());
        m_telemetry.update();
    }

    @Override
    public void end(boolean interrupted)
    {
        m_gripper.ledOff();
    }

    @Override
    public boolean isFinished() { return false; }

    /**
     * Schedules commands/command groups. Identical to CommandOpMode.schedule().
     * @param commands the command(s)/group(s) to be scheduled.
     */
    public void schedule(Command... commands)
    {
        CommandScheduler.getInstance().schedule(commands);
    }
}
