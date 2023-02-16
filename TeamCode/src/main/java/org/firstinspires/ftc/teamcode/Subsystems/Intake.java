package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public DcMotor Intake = null;

    public Intake(HardwareMap hardwareMap) {
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotor.Direction.REVERSE);
    }

    public void intake(double power) {
        Intake.setPower(power);
    }

    public void intakeOut() {
        Intake.setPower(1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}