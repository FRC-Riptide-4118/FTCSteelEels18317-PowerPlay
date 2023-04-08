package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
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
    public  Servo          gripper;
    public  DistanceSensor distanceSensor;

    public boolean Distance() {
        return (distanceSensor.getDistance(DistanceUnit.MM) <= 50);
    }

    public Gripper(HardwareMap hardwareMap) {
        gripper          = hardwareMap.get(Servo.class, "gripper");
        distanceSensor   = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        releaseCone();
    }

    /**
     * Returns whether a cone was detected by the color/distance sensor.
     * @return whether a cone was detected by the color/distance sensor.
     */
    public boolean coneDetected()
    {
        return false; // TODO implementation
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

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.MM);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}