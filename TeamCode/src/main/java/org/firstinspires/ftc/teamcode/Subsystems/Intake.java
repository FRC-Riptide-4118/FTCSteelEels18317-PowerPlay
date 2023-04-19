package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Util.MotorValuesConstants.IntakeServosConstants;

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
        intake      = hardwareMap.dcMotor.get("intake");
        intakeLeft  = hardwareMap.servo.get("intake_left");
        intakeRight = hardwareMap.servo.get("intake_right");
    }

    /**
     * Take in a cone.
     */
    public void in() { intake.setPower(1); }

    /**
     * Spit out a cone.
     */
    public void out() { intake.setPower(-0.7); }

    /**
     * Turn off the intake motor.
     */
    public void off() { intake.setPower(0); }

    /**
     * Close the intake servos.
     */
    public void close()
    {
        intakeLeft.setPosition(IntakeServosConstants.leftIn);
        intakeRight.setPosition(IntakeServosConstants.rightIn);
    }

    /**
     * Open the intake servos.
     */
    public void open()
    {
        intakeLeft.setPosition(IntakeServosConstants.leftOut);
        intakeRight.setPosition(IntakeServosConstants.rightOut);
    }
}