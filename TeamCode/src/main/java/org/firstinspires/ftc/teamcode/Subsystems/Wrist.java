package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.WristConstants;

public class Wrist extends SubsystemBase {

    private Servo m_wrist;


    public Wrist(HardwareMap hardwareMap)
    {
        m_wrist = hardwareMap.servo.get("wrist");

        // Default position
        toStart();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /* ---------------- Custom Functions ---------------- */
    /**
     * Moves the wrist to the starting position.
     */
    public void toStart() { m_wrist.setPosition(WristConstants.START); }

    /**
     * Moves the wrist to the scoring position.
     */
    public void toScoring() { m_wrist.setPosition(WristConstants.SCORE); }

    public double getPosition() { return m_wrist.getPosition(); }
}