package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Util.MotorValuesConstants.GripperConstants;

@Config
public class Gripper extends SubsystemBase {

    // Components
    public Servo gripper;
    public ColorSensor color;

    // Constants
    public static double redConeThreshold = 200;
    public static double blueConeThreshold = 200;

    public Gripper(HardwareMap hardwareMap) {
        gripper = hardwareMap.get(Servo.class, "gripper");
        color   = hardwareMap.get(ColorSensor.class, "color_cone");

        releaseCone();
    }

    /**
     * Returns whether a cone was detected by the color/distance sensor.
     * @return whether a cone was detected by the color/distance sensor.
     */
    public boolean coneDetected() { return (color.red() > redConeThreshold || color.blue() > blueConeThreshold); }

    public boolean isGripping() {
        return getPosition() == GripperConstants.grip;
    }

    public double getPosition() { return gripper.getPosition(); }

    public void gripCone()
    {
        gripper.setPosition(GripperConstants.grip);
    }

    public void releaseCone()
    {
        gripper.setPosition(GripperConstants.release);
    }

    public void toggle()
    {
        if(isGripping())    releaseCone();
        else                gripCone();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}