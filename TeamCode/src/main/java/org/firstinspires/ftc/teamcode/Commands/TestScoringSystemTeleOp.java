package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Alignment;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;
import org.firstinspires.ftc.teamcode.Subsystems.Wrist;

public class TestScoringSystemTeleOp extends CommandBase {

    private boolean startDepositMachine;
    private boolean startBeaconMachine;
    ElapsedTime depositTimer;
    ElapsedTime beaconTimer;

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
    private boolean gp1_lb_PLI  = false;
    private boolean gp1_a_PLI   = false;
    private boolean gp1_rt_PLI  = false;
    private boolean gp1_rb_PLI  = false;
    private boolean gp1_dpd_PLI = false;
    private boolean gp1_dpu_PLI = false;
    private boolean gp1_dpl_PLI = false;
    private boolean gp1_rsb_PLI = false;

    public enum DepositState {
        IDLE,
        RELEASE,
        DOWN,
        DONE
    }

    private enum BeaconState {
        IDLE,
        ARM_UP,
        PULSE,
        DONE
    }

    DepositState depositState = DepositState.IDLE;
    BeaconState beaconState = BeaconState.IDLE;

    // Constants
    private final double TRIGGER_THRESHOLD = 0.5;


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
        startDepositMachine = false;
        startBeaconMachine = false;
        depositTimer = new ElapsedTime();
        beaconTimer = new ElapsedTime();
    }

    @Override
    public void execute()
    {
        ScoringState nextScoringState = scoringState.get();
        IntakeState nextIntakeState = intakeState.get();

        boolean lb_pressed  = m_gamepad1.left_bumper;
        boolean a_pressed   = m_gamepad1.a;
        boolean rt_pressed  = m_gamepad1.right_trigger > TRIGGER_THRESHOLD;
        boolean rb_pressed  = m_gamepad1.right_bumper;
        boolean dpd_pressed = m_gamepad1.dpad_down;
        boolean dpu_pressed = m_gamepad1.dpad_up;
        boolean dpl_pressed = m_gamepad1.dpad_left;
        boolean rsb_pressed = false;//m_gamepad1.right_stick_button;

        /* ------------ Intake ------------ */
        if(intakeState == IntakeState.OFF)
        {
            m_intake.open();
            m_intake.off();

            if(rt_pressed && !gp1_rt_PLI) {
                nextIntakeState = IntakeState.IN;
                if(scoringState == ScoringState.GROUND) schedule(new ArmToStart(m_arm));
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
                schedule(new SequentialCommandGroup(
                        new GripperGripCone(m_gripper),
                        new DelayForSeconds(0.2),
                        new ArmToMiddle(m_arm)
                ));
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
//            // Manual slides reset
//            if(dpl_pressed && !gp1_dpl_PLI) {
//                m_slides.resetEncoders();
//                m_telemetry.addLine("Resetting encoders");
//            }

            if(m_gamepad1.x)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new SlidesToLow(m_slides),
                        new ArmToPreScore(m_arm),
                        new DelayForSeconds(0.2),
                        new WristToScoring(m_wrist)
                ));

                nextScoringState = ScoringState.READY;
            }

            if(m_gamepad1.y)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new DelayForSeconds(0.04),
                        new WristToScoring(m_wrist),
                        new SlidesToMedium(m_slides)
                ));

                nextScoringState = ScoringState.READY;
            }

            if(m_gamepad1.b)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new DelayForSeconds(0.04),
                        new WristToScoring(m_wrist),
                        new SlidesToHigh(m_slides)
                ));

                nextScoringState = ScoringState.READY;
            }

            if(m_gamepad1.a)
            {
                schedule(new ArmToStart(m_arm));
            }
        }
        else if(scoringState == ScoringState.READY)
        {
            if(m_gamepad1.x)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new DelayForSeconds(0.04),
                        new WristToScoring(m_wrist),
                        new SlidesToLow(m_slides)
                ));

                nextScoringState = ScoringState.READY;
            }

            if(m_gamepad1.y)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new DelayForSeconds(0.04),
                        new WristToScoring(m_wrist),
                        new SlidesToMedium(m_slides)
                ));

                nextScoringState = ScoringState.READY;
            }


            if(m_gamepad1.b)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new DelayForSeconds(0.04),
                        new WristToScoring(m_wrist),
                        new SlidesToHigh(m_slides)
                ));

                nextScoringState = ScoringState.READY;
            }

            if(rsb_pressed && !gp1_rsb_PLI)
            {
                startBeaconMachine = true;
            }

            if(a_pressed && !gp1_a_PLI)
            {
                schedule(new SequentialCommandGroup(
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
                        new DelayForSeconds(0.04),
                        new WristToScoring(m_wrist),
                        new SlidesToLow(m_slides)
                ));

                nextScoringState = ScoringState.READY;
            }

            if(m_gamepad1.y)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new DelayForSeconds(0.04),
                        new WristToScoring(m_wrist),
                        new SlidesToMedium(m_slides)
                ));

                nextScoringState = ScoringState.READY;
            }


            if(m_gamepad1.b)
            {
                schedule(new SequentialCommandGroup(
                        new AlignmentToScoring(m_alignment),
                        new ArmToPreScore(m_arm),
                        new DelayForSeconds(0.04),
                        new WristToScoring(m_wrist),
                        new SlidesToHigh(m_slides)
                ));

                nextScoringState = ScoringState.READY;
            }

            if(a_pressed && !gp1_a_PLI)
            {
                startDepositMachine = true;

                nextScoringState = ScoringState.GROUND;
            }
        }

        switch(depositState) {
            case IDLE:
                if(startDepositMachine) {
                    depositTimer.reset();
                    startDepositMachine = false;
                    depositState = DepositState.RELEASE;
                }
                break;
            case RELEASE:
                m_gripper.releaseCone();

                if(depositTimer.seconds() > 0.22) {
                    m_wrist.toStart();
                    m_slides.slidesToGround();
                    depositTimer.reset();
                    depositState = DepositState.DOWN;
                }
                break;
            case DOWN:
                if(depositTimer.seconds() > 0.25 && depositTimer.seconds() < 0.26) {
                    m_arm.armToStart();
                }

                if(depositTimer.seconds() > 1) {
                    m_alignment.up();
                    depositState = DepositState.IDLE;
                }
                break;
        }

        switch(beaconState) {
            case IDLE:
                if(startBeaconMachine) {
                    beaconTimer.reset();
                    startBeaconMachine = false;
                    beaconState = BeaconState.ARM_UP;
                }
                break;
            case ARM_UP:
                m_arm.armToPreScoreBeacon();

                if(beaconTimer.seconds() > 0.5) {
                    beaconTimer.reset();
                    beaconState = BeaconState.PULSE;
                    m_gripper.releaseCone();
                }
                break;
            case PULSE:
                if(beaconTimer.seconds() > 0.1) {
                    m_gripper.gripCone();
                }

                if(beaconTimer.seconds() > 1) {
                    beaconState = BeaconState.IDLE;
                }
                break;
            case DONE:
                if(lb_pressed) {
                    m_gripper.releaseBeacon();
                    beaconState = BeaconState.IDLE;
                }

        }


        // Update state
        scoringState = nextScoringState.get();
        intakeState = nextIntakeState.get();

        gp1_rb_PLI  = rb_pressed;
        gp1_a_PLI   = a_pressed;
        gp1_rt_PLI  = rt_pressed;
        gp1_lb_PLI  = lb_pressed;
        gp1_dpd_PLI = dpd_pressed;
        gp1_dpu_PLI = dpu_pressed;
        gp1_dpl_PLI = dpl_pressed;
        gp1_rsb_PLI = rsb_pressed;

        /* ------------ Telemetry ------------ */
        m_telemetry.addData("scoring state", scoringState.toString());
        m_telemetry.addData("intake state", intakeState.toString());
        m_telemetry.addData("left slide pos", m_slides.getLeftPosition());
        m_telemetry.addData("right slide pos", m_slides.getRightPosition());
        m_telemetry.addData("left slide target", m_slides.getLeftTarget());
        m_telemetry.addData("right slide target", m_slides.getRightTarget());
        m_telemetry.addData("rsb pressed", rsb_pressed);
        m_telemetry.addData("rsb PLI", gp1_rsb_PLI);
        m_telemetry.addData("startBeaconMachine", startBeaconMachine);
        m_telemetry.addData("beacon state", beaconState);
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