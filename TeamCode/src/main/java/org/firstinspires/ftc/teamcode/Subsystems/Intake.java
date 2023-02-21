package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public DcMotor Intake = null;

    public Intake(HardwareMap hardwareMap) {
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft =       hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight =       hardwareMap.get(Servo.class, "intakeRight");
    }

    public void intake(double power) {
        Intake.setPower(power);
    }

    public void intakeIn() {
        Intake.setPower(1);
    }

    public void intakeOut() {
        Intake.setPower(-0.5);
    }

    //Servos
    public Servo intakeLeft = null;
    public Servo intakeRight = null;

    // Intake Servo Values
    public static double IntakeLeft_out    = 0.9;
    public static double IntakeRight_out   = 1;
    public static double IntakeLeft_in     = 0.3;
    public static double IntakeRight_in    = 0.75;

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