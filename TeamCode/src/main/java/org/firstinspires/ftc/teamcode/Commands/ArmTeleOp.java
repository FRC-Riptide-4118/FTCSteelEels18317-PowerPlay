package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;

/**
 * An example command that uses an example subsystem.
 */
public class ArmTeleOp extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Arm m_arm;
    private final Gamepad m_gamepad1;
    private boolean raisingToLow = false;
    private boolean returning = false;
    private boolean raisingToMiddle = false;
    private boolean raisingToHigh = false;
    private ElapsedTime armInTimer;

    /**
     * Creates a new ExampleCommand.
     *
     * @param arm The subsystem used by this command.
     */
    public ArmTeleOp(Arm arm, Gamepad gamepad1) {
        m_arm        = arm;
        m_gamepad1   = gamepad1;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
        armInTimer = new ElapsedTime();
        armInTimer.reset();
    }

    @Override
    public void initialize() {
            m_arm.armToStart();
    }

    @Override
    public void execute() {
        /*-------Lift & Arm-------*/
        // Ground
        if(gamepad1.a) {
            returning = true;
            hardware.gripCone();
            hardware.armToStart();
            if (armInTimer.seconds() > 1.0) armInTimer.reset();
        }

        if(returning) {
            if(armInTimer.seconds() > 1.0) {
                hardware.setSlidesPower(0.7);
                hardware.slidesToStart();
                hardware.releaseCone();
                hardware.armToStart();
                hardware.armToStart();
                returning = false;
            }
        }

        // Low
        if(gamepad1.x) {
            raisingToLow = true;
            hardware.gripCone();
            hardware.setSlidesPower(1);
            hardware.slidesToLow();
        }
        if(raisingToLow) {
            if(hardware.leftSlide.getCurrentPosition() > 600) {
                hardware.armToLow();
                raisingToLow = false;
            }
        }

        // Medium
        if(gamepad1.y) {
            raisingToMiddle = true;
            hardware.gripCone();
            hardware.setSlidesPower(1);
            hardware.slidesToMedium();
        }
        if(raisingToMiddle) {
            if(hardware.leftSlide.getCurrentPosition() > 600) {
                hardware.armToMedium();
                raisingToMiddle = false;
            }
        }
aa
        // High
        if(gamepad1.b) {
            raisingToHigh = true;
            hardware.gripCone();
            hardware.setSlidesPower(1);
            hardware.slidesToHigh();
        }
        if(raisingToHigh) {
            if(hardware.leftSlide.getCurrentPosition() > 600) {
                hardware.armToHigh();
                raisingToHigh = false;
            }
        }
    }

//    @Override
//    public boolean isFinished() {
//        return false;
//    }

}
