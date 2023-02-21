package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.IntakeServosConstants;

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

    public void intakeServoIn() {
        intakeLeft.setPosition(IntakeServosConstants.IntakeLeft_in);
        intakeRight.setPosition(IntakeServosConstants.IntakeRight_in);
    }

    public void intakeServoOut() {
        intakeLeft.setPosition(IntakeServosConstants.IntakeLeft_out);
        intakeRight.setPosition(IntakeServosConstants.IntakeRight_out);
    }

    public boolean intakeServosIn() {
        return intakeLeft.getPosition() == IntakeServosConstants.IntakeLeft_in;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}