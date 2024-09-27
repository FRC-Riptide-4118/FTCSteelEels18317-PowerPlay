package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Util.MotorValuesConstants.ArmConstants;

public class Arm extends SubsystemBase {
    public Servo armRight;
    public Servo armLeft;

    public Arm(HardwareMap hardwareMap) {
        armRight    = hardwareMap.get(Servo.class, "arm_right");
        armLeft     = hardwareMap.get(Servo.class, "arm_left");

        toStart();
    }

    public double getPosition() {
        return armRight.getPosition();
    }

    public void toStart() {
        armRight.setPosition(ArmConstants.start);
        armLeft.setPosition(ArmConstants.start);
    }

    public void toStack() {
        armRight.setPosition(ArmConstants.stack);
        armLeft.setPosition(ArmConstants.stack);
    }

    public void armToMiddle() {
        armRight.setPosition(ArmConstants.middle);
        armLeft.setPosition(ArmConstants.middle);
    }
    
    public void armToPreScore()
    {
        armRight.setPosition(ArmConstants.preScore);
        armLeft.setPosition(ArmConstants.preScore);
    }

    public void armToScore() {
        armRight.setPosition(ArmConstants.score);
        armLeft.setPosition(ArmConstants.score);
    }

    @Override
    public void periodic() {

    }
}