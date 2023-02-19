package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public Servo arm1 = null;
    public Servo arm2 = null;

    // Arm Servo Values
    public static double Arm1_Start        = 0.15;
    public static double Arm2_Start        = 0.85;
    public static double Arm1_Scoring      = 0.8;
    public static double Arm2_Scoring      = 0.2;
    public static double Arm1_Cone1        = 0;
    public static double Arm2_Cone1        = 1;
    public static double Arm1_Cone2        = 0.05;
    public static double Arm2_Cone2        = 0.95;
    public static double Arm1_Cone3        = 0.1;
    public static double Arm2_Cone3        = 0.9;
    public static double Arm1_Cone4        = 0.15;
    public static double Arm2_Cone4        = 0.85;
    public static double Arm1_Cone5        = 0.2;
    public static double Arm2_Cone5        = 0.8;

    private int m_counter = 0;

    private final int NUM_STATES = 5;

    public Arm(HardwareMap hardwareMap) {
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");

        // Arm servos
        armToStart();

        // Wiggle the arm servos so they (hopefully) don't
        // misbehave on the first two real commands
        arm1.setPosition(Arm1_Start + 0.2);
        arm2.setPosition(Arm2_Start - 0.2);
        armToStart();

    }

    public boolean ArmScoring() {
        return arm1.getPosition() == Arm1_Scoring;
    }

    public void armToStart() {
        arm1.setPosition(Arm1_Start);
        arm2.setPosition(Arm2_Start);
    }

    public void armToCone1()
    {
        arm1.setPosition(Arm1_Cone1);
        arm2.setPosition(Arm2_Cone1);
    }

    public void armToCone1Wiggle()
    {
        arm1.setPosition(Arm1_Cone1 - 0.2);
        arm2.setPosition(Arm2_Cone1 + 0.2);
    }

    public void armToCone2()
    {
        arm1.setPosition(Arm1_Cone2);
        arm2.setPosition(Arm2_Cone2);
    }

    public void armToCone3()
    {
        arm1.setPosition(Arm1_Cone3);
        arm2.setPosition(Arm2_Cone3);
    }

    public void armToCone4()
    {
        arm1.setPosition(Arm1_Cone4);
        arm2.setPosition(Arm2_Cone4);
    }

    public void armToCone5()
    {
        arm1.setPosition(Arm1_Cone5);
        arm2.setPosition(Arm2_Cone5);
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

    public void armScoring()
    {
        arm1.setPosition(Arm1_Scoring);
        arm2.setPosition(Arm2_Scoring);
    }

    public void armWiggle()
    {
        armToStart();
        arm1.setPosition(Arm1_Start + 0.01);
        arm2.setPosition(Arm2_Start + 0.01);
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

    public void incrementCounter(){
        m_counter = (m_counter + 1) % 5;
    }

    @Override
    public void periodic() {

    }
}