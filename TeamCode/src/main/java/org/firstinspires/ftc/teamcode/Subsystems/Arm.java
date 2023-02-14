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
    public static double Arm1_Start        = 0.2;
    public static double Arm2_Start        = 0.8;
    public static double Arm1_Scoring      = 0.85;
    public static double Arm2_Scoring      = 0.15;
    public static double Arm1_Front        = 0.85;
    public static double Arm2_Front        = 0.15;
    public static double Arm1_Cone1        = 0;
    public static double Arm2_Cone1        = 1;

    public Arm(HardwareMap hardwareMap) {
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");

        // Arm servos
        armToStart();

        // Wiggle the arm servos so they (hopefully) don't
        // misbehave on the first two real commands
        arm1.setPosition(Arm1_Start + 0.01);
        arm2.setPosition(Arm2_Start + 0.01);
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}