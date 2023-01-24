package org.firstinspires.ftc.teamcode.BlakeStuff.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain extends SubsystemBase {

    // Hardware elements
    private final DcMotor flDrive;
    private final DcMotor frDrive;
    private final DcMotor rlDrive;
    private final DcMotor rrDrive;

    /**
     * Constructor for the subsystem. This creates
     * a new instance of the subsystem in whichever
     * OpMode is instantiating it.
     */
    public Drivetrain(HardwareMap hwMap)
    {
        flDrive = hwMap.dcMotor.get("front_left_wheel");
        frDrive = hwMap.dcMotor.get("front_right_wheel");
        rlDrive = hwMap.dcMotor.get("rear_left_wheel");
        rrDrive = hwMap.dcMotor.get("rear_right_wheel");
    }

    /**
     * Runs once each time the command scheduler runs.
     */
    @Override
    public void periodic()
    {

    }

    /**
     * Set drive motor powers based on Mecanum-drive-style inputs.
     *
     * @param drive the amount of forward motion
     * @param strafe the amount of sideways motion
     * @param twist the amount of rotational motion
     */
    public void setMecanumPower(double drive, double strafe, double twist)
    {
        flDrive  .setPower(drive + strafe + twist);
        frDrive  .setPower(drive - strafe - twist);
        rlDrive  .setPower(drive - strafe + twist);
        rrDrive  .setPower(drive + strafe - twist);
    }

    /**
     * Stop all motors.
     */
    public void stop()
    {
        flDrive.setPower(0);
        frDrive.setPower(0);
        rlDrive.setPower(0);
        rrDrive.setPower(0);
    }


    /**
     * Returns whether any drive motors are still running.
     * @return whether any drive motors are still running.
     */
    public boolean isBusy()
    {
        return flDrive.isBusy() || frDrive.isBusy() || rlDrive.isBusy() || rrDrive.isBusy();
    }
}
