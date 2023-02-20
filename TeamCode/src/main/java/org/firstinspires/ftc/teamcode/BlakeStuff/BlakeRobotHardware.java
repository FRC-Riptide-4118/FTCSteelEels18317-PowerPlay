package org.firstinspires.ftc.teamcode.BlakeStuff;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm1;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Arm2;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.SlidesConstants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * @Author Blake Sanders
 * Created Jan 21, 2023
 *
 * This class includes all of the hardware on the robot (motors, sensors, servos, etc.).
 *      By putting it in this class, we can use the exact same hardware across different
 *      OpModes by creating an instance of this class in each OpMode and then calling the
 *      init() method. We can get more complex by only initializing certain subsystems or
 *      components (e.g., only init the camera in a camera test OpMode), as well as by
 *      splitting the robot into subsystems (e.g. arm, lift, etc.) rather than having
 *      everything in here. You can also get it access to telemetry, if you want to
 *      handle that in here.
 */
public class BlakeRobotHardware
{
    // Hardware map (this will be set to the hardwareMap of the current
    // OpMode once the RobotHardware class is instantiated
    HardwareMap hardwareMap;

    // Hardware components (motors, sensors, servos, etc)
    public DcMotor frontLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontRight = null;
    public DcMotor rearLeft = null;
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;
    public Servo Gripper = null;
    public Servo arm1 = null;
    public Servo arm2 = null;

    // Robot constants
    int HD_COUNTS_PER_REV = 28;
    int DRIVE_GEAR_REDUCTION = 20;
    double WHEEL_CIRCUMFERENCE_MM = 101.6 * Math.PI;
    double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    /**
     * Simple constructor to set the hardware map for this
     * hardware class to the one in the OpMode that is
     * currently being run
     *
     * @param hwMap the hardware map in the OpMode being run
     */
    public BlakeRobotHardware(HardwareMap hwMap)
    {
        hardwareMap = hwMap;
    }

    /**
     * Initialize the hardware and do any necessary setup. This could include
     * setting motor directions and run modes, starting servo positions, or
     * starting a USB camera for example.
     */
    public void init()
    {
        /*------- Initialize hardware -------*/
        frontLeft =     hardwareMap.get(DcMotor.class, "front_left_wheel");
        rearLeft =      hardwareMap.get(DcMotor.class, "rear_left_wheel");
        frontRight =    hardwareMap.get(DcMotor.class, "front_right_wheel");
        rearRight =     hardwareMap.get(DcMotor.class, "rear_right_wheel");
        leftSlide =     hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlide =    hardwareMap.get(DcMotorEx.class, "right_slide");
        Gripper =       hardwareMap.get(Servo.class, "Gripper");
        arm1 =          hardwareMap.get(Servo.class, "arm1");
        arm2 =          hardwareMap.get(Servo.class, "arm2");
        // I find things like ^--this easier to read when I tab similar lines to
        // match up with each other. Do what works for you (:

        /*------- Do hardware setup -------*/

        // Drive motors
        frontLeft.  setDirection(DcMotor.Direction.REVERSE);
        rearLeft.   setDirection(DcMotor.Direction.REVERSE);
        frontRight. setDirection(DcMotor.Direction.FORWARD);
        rearRight.  setDirection(DcMotor.Direction.FORWARD);

        // Slide motors
        leftSlide.  setDirection(DcMotor.Direction.REVERSE);
        rightSlide. setDirection(DcMotor.Direction.REVERSE);

        leftSlide.  setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide. setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.  setTargetPosition(SlidesConstants.Start);
        rightSlide. setTargetPosition(SlidesConstants.Start);

        leftSlide.  setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide. setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Arm servos
        arm1.setPosition(Arm1.Start);
        arm2.setPosition(Arm2.Start);

        // Wiggle the arm servos so they (hopefully) don't
        // misbehave on the first two real commands
        arm1.setPosition(Arm1.Start + 0.01);
        arm2.setPosition(Arm2.Start + 0.01);
        arm1.setPosition(Arm1.Start);
        arm2.setPosition(Arm2.Start);
    }

    /*------- Define any other methods down here! -------*/


    // Here's a simple example:
    /**
     * Sets the arm to a new position.
     *
     * @param position1 the new position for the arm1 servo
     * @param position2 the new position for the arm2 servo
     */
    public void setArmPosition(double position1, double position2)
    {
        arm1.setPosition(position1);
        arm2.setPosition(position2);
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

    // InchesToCounts
    public int inchesToCounts(double inches) {
        return (int) (inches * DRIVE_COUNTS_PER_IN);
    }
}
