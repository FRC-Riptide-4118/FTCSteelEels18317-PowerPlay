package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.IntakeServosConstants;

public class Intake extends SubsystemBase {
    public DcMotor intake;
    public Servo intakeLeft;
    public Servo intakeRight;

    /**
     * Default constructor to initialize intake servos and motors.
     * @param hardwareMap the current OpMode's hardware map.
     */
    public Intake(HardwareMap hardwareMap)
    {
        intake      = hardwareMap.get(DcMotor.class, "intake");
        intakeLeft  = hardwareMap.get(Servo.class, "intake_left");
        intakeRight = hardwareMap.get(Servo.class, "intake_right");
    }

    /**
     * Take in a cone.
     */
    public void in() { intake.setPower(1); }

    /**
     * Spit out a cone.
     */
    public void out() { intake.setPower(-0.5); }

    /**
     * Turn off the intake motor.
     */
    public void off() { intake.setPower(0); }

    /**
     * Close the intake servos.
     */
    public void close()
    {
        intakeLeft.setPosition(IntakeServosConstants.INTAKE_LEFT_IN);
        intakeRight.setPosition(IntakeServosConstants.INTAKE_RIGHT_IN);
    }

    /**
     * Open the intake servos.
     */
    public void open()
    {
        intakeLeft.setPosition(IntakeServosConstants.INTAKE_LEFT_OUT);
        intakeRight.setPosition(IntakeServosConstants.INTAKE_RIGHT_OUT);
    }
}