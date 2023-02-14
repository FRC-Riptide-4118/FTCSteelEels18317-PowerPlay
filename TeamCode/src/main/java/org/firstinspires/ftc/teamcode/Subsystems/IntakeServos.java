package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class IntakeServos extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public Servo intakeLeft = null;
    public Servo intakeRight = null;

    // Intake Servo Values
    public static double IntakeLeft_out    = 0.9;
    public static double IntakeRight_out   = 0.28;
    public static double IntakeLeft_in     = 0.3;
    public static double IntakeRight_in    = 0.75;

    public IntakeServos(HardwareMap hardwareMap) {
        intakeLeft =       hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight =       hardwareMap.get(Servo.class, "intakeRight");
    }

    public void intakeServoIn() {
        intakeLeft.setPosition(IntakeLeft_in);
        intakeRight.setPosition(IntakeRight_in);
    }

    public void intakeServoOut() {
        intakeLeft.setPosition(IntakeLeft_out);
        intakeRight.setPosition(IntakeRight_out);
    }

    public boolean intakeServosIn() {
        return intakeLeft.getPosition() == IntakeLeft_in;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}