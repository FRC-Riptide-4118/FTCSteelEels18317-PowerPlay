package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Intake1_in;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.IntakeLeft_out;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Intake2_in;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.IntakeRight_out;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeServos extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public Servo intakeLeft = null;
    public Servo intakeRight = null;

    public IntakeServos(HardwareMap hardwareMap) {
        intakeLeft =       hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight =       hardwareMap.get(Servo.class, "intakeRight");
    }

    public void intakeServoIn() {
        intakeLeft.setPosition(Intake1_in);
        intakeRight.setPosition(Intake2_in);
    }

    public void intakeServoOut() {
        intakeLeft.setPosition(IntakeLeft_out);
        intakeRight.setPosition(IntakeRight_out);
    }

    public boolean intakeServosIn() {
        return intakeLeft.getPosition() == Intake1_in;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}