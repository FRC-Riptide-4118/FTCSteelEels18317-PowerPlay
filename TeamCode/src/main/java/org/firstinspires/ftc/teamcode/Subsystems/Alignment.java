package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants;

public class Alignment extends SubsystemBase {

    private Servo m_alignment;


    public Alignment(HardwareMap hardwareMap) {
        m_alignment = hardwareMap.servo.get("alignment");

        up();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /* ---------------- Custom Functions ---------------- */

    public void up()
    {
        m_alignment.setPosition(MotorValuesConstants.AlignmentConstants.UP);
    }

    public void score()
    {
        m_alignment.setPosition(MotorValuesConstants.AlignmentConstants.SCORE);
    }

    public void down()
    {
        m_alignment.setPosition(MotorValuesConstants.AlignmentConstants.DOWN);
    }
}