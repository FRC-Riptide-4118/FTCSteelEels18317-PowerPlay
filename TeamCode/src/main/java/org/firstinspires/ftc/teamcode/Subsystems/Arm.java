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
        armRight    = hardwareMap.get(Servo.class, "arm1");
        armLeft     = hardwareMap.get(Servo.class, "arm2");

        // Arm servos
        armToStart();
    }

    
    public void armToStart() {
        armRight.setPosition(ArmRightConstants.START);
        armLeft.setPosition(ArmLeftConstants.START);
    }

    public void armToStack() {
        armRight.setPosition(ArmRightConstants.STACK);
        armLeft.setPosition(ArmLeftConstants.STACK);
    }

    public void armToMiddle() {
        armRight.setPosition(ArmRightConstants.MIDDLE);
        armLeft.setPosition(ArmLeftConstants.MIDDLE);
    }
    
    public void armToSCORE() // FIXME should we rename to "armToSCORE()" @Lohan @Elinora?
    {
        armRight.setPosition(MotorValuesConstants.ArmRightConstants.SCORE);
        armLeft.setPosition(MotorValuesConstants.ArmLeftConstants.SCORE);
    }

    public void armWiggle()
    {
        armToStart();
        armRight.setPosition(MotorValuesConstants.ArmRightConstants.SCORE + 0.01);
        armLeft.setPosition(MotorValuesConstants.ArmLeftConstants.SCORE + 0.01);
        armToStart();
    }

    public void armToCone1()
    {
        armRight.setPosition(MotorValuesConstants.Arm1Constants.CONE_1);
        armLeft.setPosition(MotorValuesConstants.Arm2Constants.CONE_1);
    }

    public void armToCone1Wiggle()
    {
        armRight.setPosition(MotorValuesConstants.Arm1Constants.CONE_1 - 0.2);
        armLeft.setPosition(MotorValuesConstants.Arm2Constants.CONE_1 + 0.2);
    }

    public void armToCone2()
    {
        armRight.setPosition(MotorValuesConstants.Arm1Constants.CONE_2);
        armLeft.setPosition(MotorValuesConstants.Arm2Constants.CONE_2);
    }

    public void armToCone3()
    {
        armRight.setPosition(MotorValuesConstants.Arm1Constants.CONE_3);
        armLeft.setPosition(MotorValuesConstants.Arm2Constants.CONE_3);
    }

    public void armToCone4()
    {
        armRight.setPosition(MotorValuesConstants.Arm1Constants.CONE_4);
        armLeft.setPosition(MotorValuesConstants.Arm2Constants.CONE_4);
    }

    public void armToCone5()
    {
        armRight.setPosition(MotorValuesConstants.Arm1Constants.CONE_5);
        armLeft.setPosition(MotorValuesConstants.Arm2Constants.CONE_5);
    }

    public void armToDrop() {
        armRight.setPosition(ArmRightConstants.SCORE - 0.18);
        armLeft.setPosition(ArmLeftConstants.SCORE + 0.18);
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