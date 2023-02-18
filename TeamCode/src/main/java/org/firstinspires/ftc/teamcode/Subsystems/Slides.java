package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Slides extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;

    //Slides Encoder Values
    public static int Slides_Start         = -5;
    public static int Slides_Ground        = 0;
    public static int TeleOpSlides_Ground  = 0;
    public static int Slides_Low           = 600;
    public static int Slides_Medium        = 1200;
    public static int Slides_High          = 1850;

    public Slides(HardwareMap hardwareMap) {
        leftSlide =  hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_slide");

        // Slide motors
        leftSlide.  setDirection(DcMotor.Direction.REVERSE);
        rightSlide. setDirection(DcMotor.Direction.FORWARD);

        leftSlide.  setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide. setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.  setTargetPosition(Slides_Start);
        rightSlide. setTargetPosition(Slides_Start);

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
        leftSlide.setTargetPosition(Slides_Ground);
        rightSlide.setTargetPosition(Slides_Ground);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public void slidesToLow()
    {
        leftSlide.setTargetPosition(Slides_Low);
        rightSlide.setTargetPosition(Slides_Low);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public void slidesToMedium()
    {
        leftSlide.setTargetPosition(Slides_Medium);
        rightSlide.setTargetPosition(Slides_Medium);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public void slidesToMediumAuto()
    {
        leftSlide.setTargetPosition(Slides_Medium+100);
        rightSlide.setTargetPosition(Slides_Medium);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public void slidesToHigh()
    {
        leftSlide.setTargetPosition(Slides_High);
        rightSlide.setTargetPosition(Slides_High);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    public boolean slidesAreBusy()
    {
        return leftSlide.isBusy() || rightSlide.isBusy();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}