package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Simple drivetrain with Mecanum-style teleop controls.
 */
public class Drivetrain extends SubsystemBase {
    public DcMotor frontLeft;
    public DcMotor rearRight;
    public DcMotor frontRight;
    public DcMotor rearLeft;

    public Drivetrain(HardwareMap hardwareMap) {
        frontLeft   = hardwareMap.get(DcMotor.class, "front_left_wheel");
        rearLeft    = hardwareMap.get(DcMotor.class, "rear_left_wheel");
        frontRight  = hardwareMap.get(DcMotor.class, "front_right_wheel");
        rearRight   = hardwareMap.get(DcMotor.class, "rear_right_wheel");

        // Drive motors
        frontLeft.  setDirection(DcMotor.Direction.REVERSE);
        rearLeft.   setDirection(DcMotor.Direction.REVERSE);
        frontRight. setDirection(DcMotor.Direction.FORWARD);
        rearRight.  setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * Set drive motor powers based on Mecanum-drive-style inputs.
     *
     * @param drive the amount of forward motion
     * @param strafe the amount of sideways motion
     * @param twist the amount of rotational motion
     * @param slowMode the amount of slowing to apply
     */
    public void setMecanumPower(double drive, double strafe, double twist, double slowMode)
    {
        frontLeft   .setPower((drive + strafe + twist) * slowMode);
        frontRight  .setPower((drive - strafe - twist) * slowMode);
        rearLeft    .setPower((drive - strafe + twist) * slowMode);
        rearRight   .setPower((drive + strafe - twist) * slowMode);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}