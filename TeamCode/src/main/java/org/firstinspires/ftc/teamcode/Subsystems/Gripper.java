package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.GripperConstants;

@Config
public class Gripper extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public Servo        gripper;
    public ColorSensor  color;

    public Gripper(HardwareMap hardwareMap) {
        gripper = hardwareMap.get(Servo.class, "gripper");
        color   = hardwareMap.get(ColorSensor.class, "color_cone");

        releaseCone();
        ledOn();
    }

    public void ledOn() { color.enableLed(true); }
    public void ledOff() { color.enableLed(false); } // FIXME currently not working

    /**
     * Returns whether a cone was detected by the color/distance sensor.
     * @return whether a cone was detected by the color/distance sensor.
     */
    public boolean coneDetected()
    {
        // FIXME check threshold values!!! Red was determined empirically under even, cool, LED lighting
        return color.red() > 200 || color.blue() > 200;
    }

    public boolean isGripping() {
        return gripper.getPosition() == GripperConstants.GRIP;
    }
    public double getPosition() { return gripper.getPosition(); }

    public void gripCone()
    {
        gripper.setPosition(GripperConstants.GRIP);
    }

    public void releaseCone()
    {
        gripper.setPosition(GripperConstants.RELEASE);
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