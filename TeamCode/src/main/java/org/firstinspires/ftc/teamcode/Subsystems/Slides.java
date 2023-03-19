package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.SlidesConstants;

public class Slides extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;


    private final double SAFE_SLIDES_HEIGHT = 600; // encoder counts

    public Slides(HardwareMap hardwareMap) {
        leftSlide =  hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_slide");

        // Slide motors
        leftSlide.  setDirection(DcMotor.Direction.REVERSE);
        rightSlide. setDirection(DcMotor.Direction.FORWARD);

        leftSlide.  setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide. setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.  setTargetPosition(SlidesConstants.START);
        rightSlide. setTargetPosition(SlidesConstants.START);

        leftSlide.  setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide. setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // PID Values
        leftSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(5, 0, 0, 0));
        rightSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(5, 0, 0, 0));
    }

    public void setSlidesPower(double power)
    {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public void slidesToStart()
    {
        leftSlide.setTargetPosition(SlidesConstants.START);
        rightSlide.setTargetPosition(SlidesConstants.START);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public void slidesToLow()
    {
        leftSlide.setTargetPosition(SlidesConstants.LOW);
        rightSlide.setTargetPosition(SlidesConstants.LOW);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public void slidesToGround()
    {
        leftSlide.setTargetPosition(SlidesConstants.GROUND);
        rightSlide.setTargetPosition(SlidesConstants.GROUND);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public void slidesToStack()
    {
        leftSlide.setTargetPosition(SlidesConstants.STACK);
        rightSlide.setTargetPosition(SlidesConstants.STACK);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public void slidesToMedium()
    {
        leftSlide.setTargetPosition(SlidesConstants.MEDIUM);
        rightSlide.setTargetPosition(SlidesConstants.MEDIUM);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public void slidesToHigh()
    {
        leftSlide.setTargetPosition(SlidesConstants.HIGH);
        rightSlide.setTargetPosition(SlidesConstants.HIGH);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public void slidesDrop()
    {
        leftSlide.setTargetPosition(leftSlide.getCurrentPosition()-300);
        rightSlide.setTargetPosition(rightSlide.getCurrentPosition()-300);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public void slidesUp()
    {
        leftSlide.setTargetPosition(leftSlide.getCurrentPosition()+300);
        rightSlide.setTargetPosition(rightSlide.getCurrentPosition()+300);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public boolean slidesAreBusy()
    {
        return leftSlide.isBusy() || rightSlide.isBusy();
    }

    public boolean atSafeHeight() { return (leftSlide.getCurrentPosition() - SAFE_SLIDES_HEIGHT) >= 0; }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}