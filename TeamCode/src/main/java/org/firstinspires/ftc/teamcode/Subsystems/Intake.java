package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Grab;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Gripper_Release;

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
    }

    public void intake(double power) {
        Intake.setPower(power);
    }

    public void intakeOut() {
        Intake.setPower(-.5);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}