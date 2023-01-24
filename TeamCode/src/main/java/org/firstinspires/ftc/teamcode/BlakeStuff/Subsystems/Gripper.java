package org.firstinspires.ftc.teamcode.BlakeStuff.Subsystems;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Grab;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Release;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper extends SubsystemBase {

    // Hardware elements
    private final Servo gripper;

    /**
     * Constructor for the subsystem. This creates
     * a new instance of the subsystem in whichever
     * OpMode is instantiating it.
     */
    public Gripper(HardwareMap hwMap)
    {
        gripper = hwMap.get(Servo.class, "Gripper");
    }

    /**
     * Runs once each time the command scheduler runs.
     */
    @Override
    public void periodic()
    {

    }

    /**
     * Grip a cone.
     */
    public void grip()
    {
        gripper.setPosition(Gripper_Grab);
    }

    /**
     * Release a cone.
     */
    public void release()
    {
        gripper.setPosition(Gripper_Release);
    }
}
