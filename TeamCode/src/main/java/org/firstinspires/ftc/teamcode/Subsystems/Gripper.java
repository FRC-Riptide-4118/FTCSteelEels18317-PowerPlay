package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Grab;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Release;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.ml.distance.DistanceMeasure;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Gripper extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public  Servo          gripper        = null;
    public  DistanceSensor distanceSensor = null;

    public boolean Distance() {
        return distanceSensor.getDistance(DistanceUnit.CM) <= 1.5;
    }

    public Gripper(HardwareMap hardwareMap) {
        gripper          = hardwareMap.get(Servo.class, "Gripper");
        distanceSensor   = hardwareMap.get(DistanceSensor.class, "distance_sensor");
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