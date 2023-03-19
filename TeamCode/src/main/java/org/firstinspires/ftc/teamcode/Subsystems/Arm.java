package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1Constants;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2Constants;

import org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants;

public class Arm extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public Servo arm1 = null;
    public Servo arm2 = null;

    private int m_counter = 0;

    private final int ENUM_STATES = 5;

    public Arm(HardwareMap hardwareMap) {
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");

        // Arm servos
        armToStart();

        // Wiggle the arm servos so they (hopefully) don't
        // misbehave on the first two real commands
//        arm1.setPosition(MotorValuesConstants.Arm1Constants.Start + 0.2);
//        arm2.setPosition(MotorValuesConstants.Arm2Constants.Start - 0.2);
//        armToStart();

    }

    public boolean ArmScoring() {
        return arm1.getPosition() == Arm1Constants.SCORING;
    }

    public void armToStart() {
        arm1.setPosition(Arm1Constants.START);
        arm2.setPosition(Arm2Constants.START);
    }

    public void armToCone1()
    {
        arm1.setPosition(Arm1Constants.CONE_1);
        arm2.setPosition(Arm2Constants.CONE_1);
    }

    public void armToCone1Wiggle()
    {
        arm1.setPosition(MotorValuesConstants.Arm1Constants.CONE_1 - 0.2);
        arm2.setPosition(MotorValuesConstants.Arm2Constants.CONE_1 + 0.2);
    }

    public void armToCone2()
    {
        arm1.setPosition(MotorValuesConstants.Arm1Constants.CONE_2);
        arm2.setPosition(MotorValuesConstants.Arm2Constants.CONE_2);
    }

    public void armToCone3()
    {
        arm1.setPosition(MotorValuesConstants.Arm1Constants.CONE_3);
        arm2.setPosition(MotorValuesConstants.Arm2Constants.CONE_3);
    }

    public void armToCone4()
    {
        arm1.setPosition(MotorValuesConstants.Arm1Constants.CONE_4);
        arm2.setPosition(MotorValuesConstants.Arm2Constants.CONE_4);
    }

    public void armToCone5()
    {
        arm1.setPosition(MotorValuesConstants.Arm1Constants.CONE_5);
        arm2.setPosition(MotorValuesConstants.Arm2Constants.CONE_5);
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

    public void armScoring() // FIXME should we rename to "armToScoring()" @Lohan @Elinora?
    {
        arm1.setPosition(MotorValuesConstants.Arm1Constants.SCORING);
        arm2.setPosition(MotorValuesConstants.Arm2Constants.SCORING);
    }

    public void armWiggle()
    {
        armToStart();
        arm1.setPosition(MotorValuesConstants.Arm1Constants.SCORING + 0.01);
        arm2.setPosition(MotorValuesConstants.Arm2Constants.SCORING + 0.01);
        armToStart();
    }

    public double getPosition(int side) {

        switch(side) {

            case 1:
                return arm1.getPosition();
            case 2:
                return arm2.getPosition();
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