package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Grab;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Release;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public Servo gripper = null;

    public Gripper(HardwareMap hardwareMap) {
        gripper =       hardwareMap.get(Servo.class, "Gripper");
    }

    public boolean isGripping() {
        return gripper.getPosition() == Gripper_Grab;
    }

    public void gripCone()
    {
        gripper.setPosition(Gripper_Grab);
    }

    public void releaseCone()
    {
        gripper.setPosition(Gripper_Release);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}