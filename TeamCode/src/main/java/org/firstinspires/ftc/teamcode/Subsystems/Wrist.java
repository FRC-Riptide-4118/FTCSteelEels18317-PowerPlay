package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.MotorValuesConstants.WristConstants;

public class Wrist extends SubsystemBase {

    private Servo m_wrist;


    public Wrist(HardwareMap hardwareMap)
    {
        m_wrist = hardwareMap.servo.get("wrist");

        // Default position
        toStart();
    }

    /**
     * Moves the wrist to the starting position.
     */
    public void toStart() { m_wrist.setPosition(WristConstants.start); }

    /**
     * Moves the wrist to the scoring position.
     */
    public void toScoring() { m_wrist.setPosition(WristConstants.score); }

    /**
     * Returns the current position of the wrist servo.
     * @return the current position of the wrist servo.
     */
    public double getPosition() { return m_wrist.getPosition(); }


    @Override
    public void periodic() {

    }
}