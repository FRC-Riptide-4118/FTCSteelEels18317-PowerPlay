package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.ArmLeftConstants;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.ArmRightConstants;

import org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants;

public class Arm extends SubsystemBase {
    public Servo armRight   = null;
    public Servo armLeft    = null;

    private int m_counter = 0;

    private final int ENUM_STATES = 5;

    public Arm(HardwareMap hardwareMap) {
        armRight    = hardwareMap.get(Servo.class, "arm_right");
        armLeft     = hardwareMap.get(Servo.class, "arm_left");

        // Arm servos
        armToStart();
    }

    public void toggleConeFlip() {
        if(armRight.getPosition() == ArmRightConstants.START)
        {
            armRight.setPosition(ArmRightConstants.CONE_FLIP);
            armLeft.setPosition(ArmLeftConstants.CONE_FLIP);

        }
        else
        {
            armRight.setPosition(ArmRightConstants.START);
            armLeft.setPosition(ArmLeftConstants.START);
        }
    }

    public double getPosition() {
        return armRight.getPosition();
    }

    
    public void armToStart() {
        armRight.setPosition(ArmRightConstants.START);
        armLeft.setPosition(ArmLeftConstants.START);
    }

    public void armToStack() {
        armRight.setPosition(ArmRightConstants.STACK);
        armLeft.setPosition(ArmLeftConstants.STACK);
    }

    public void armOffGround() {
        armRight.setPosition(ArmRightConstants.OFF_GROUND);
        armLeft.setPosition(ArmLeftConstants.OFF_GROUND);
    }

    public void armToMiddle() {
        armRight.setPosition(ArmRightConstants.MIDDLE);
        armLeft.setPosition(ArmLeftConstants.MIDDLE);
    }
    
    public void armToPreScore()
    {
        armRight.setPosition(MotorValuesConstants.ArmRightConstants.PRE_SCORE);
        armLeft.setPosition(MotorValuesConstants.ArmLeftConstants.PRE_SCORE);
    }

    public void armWiggle()
    {
        armToStart();
        armRight.setPosition(MotorValuesConstants.ArmRightConstants.SCORE + 0.01);
        armLeft.setPosition(MotorValuesConstants.ArmLeftConstants.SCORE + 0.01);
        armToStart();
    }

    @Deprecated
    public void armToCone1()
    {
        armRight.setPosition(MotorValuesConstants.Arm1Constants.CONE_1);
        armLeft.setPosition(MotorValuesConstants.Arm2Constants.CONE_1);
    }

    @Deprecated
    public void armToCone1Wiggle()
    {
        armRight.setPosition(MotorValuesConstants.Arm1Constants.CONE_1 - 0.2);
        armLeft.setPosition(MotorValuesConstants.Arm2Constants.CONE_1 + 0.2);
    }

    @Deprecated
    public void armToCone2()
    {
        armRight.setPosition(MotorValuesConstants.Arm1Constants.CONE_2);
        armLeft.setPosition(MotorValuesConstants.Arm2Constants.CONE_2);
    }

    @Deprecated
    public void armToCone3()
    {
        armRight.setPosition(MotorValuesConstants.Arm1Constants.CONE_3);
        armLeft.setPosition(MotorValuesConstants.Arm2Constants.CONE_3);
    }

    @Deprecated
    public void armToCone4()
    {
        armRight.setPosition(MotorValuesConstants.Arm1Constants.CONE_4);
        armLeft.setPosition(MotorValuesConstants.Arm2Constants.CONE_4);
    }

    @Deprecated
    public void armToCone5()
    {
        armRight.setPosition(MotorValuesConstants.Arm1Constants.CONE_5);
        armLeft.setPosition(MotorValuesConstants.Arm2Constants.CONE_5);
    }

    public void armToScore() {
        armRight.setPosition(ArmRightConstants.SCORE);
        armLeft.setPosition(ArmLeftConstants.SCORE);
    }
    public void armToPreScoreBeacon() {
        armRight.setPosition(ArmRightConstants.PRE_SCORE_BEACON);
        armLeft.setPosition(ArmLeftConstants.PRE_SCORE_BEACON);
    }

    public void armToPosition()
    {
        if (m_counter == 0)
            armToCone1();
        if (m_counter == 1)
            armToCone2();
        if (m_counter == 2)
            armToCone3();
        if (m_counter == 3)
            armToCone4();
        if (m_counter == 4)
            armToCone5();
    }

    public boolean ArmSCORE() {
        return armRight.getPosition() == ArmRightConstants.SCORE;
    }
        public double getPosition(int side) {

        switch(side) {

            case 1:
                return armRight.getPosition();
            case 2:
                return armLeft.getPosition();
            default:
                return 0;

        }

    }

    public void setValue(){
        m_counter = (m_counter + 1) % 5;
    }

    @Override
    public void periodic() {

    }
}