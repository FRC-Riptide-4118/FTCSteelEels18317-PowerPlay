package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_Ground;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_High;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_Low;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_Medium;
import static org.firstinspires.ftc.teamcode.TeleOp.MotorValuesConstants.Slides_Start;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Slides extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public DcMotorEx leftSlide = null;
    public DcMotorEx rightSlide = null;

    public Slides(HardwareMap hardwareMap) {
        leftSlide =  hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_slide");

        // Slide motors
        leftSlide.  setDirection(DcMotor.Direction.REVERSE);
        rightSlide. setDirection(DcMotor.Direction.REVERSE);

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
    }

    public void slidesToLow()
    {
        leftSlide.setTargetPosition(Slides_Low);
        rightSlide.setTargetPosition(Slides_Low);
    }

    public void slidesToMedium()
    {
        leftSlide.setTargetPosition(Slides_Medium);
        rightSlide.setTargetPosition(Slides_Medium);
    }

    public void slidesToHigh()
    {
        leftSlide.setTargetPosition(Slides_High);
        rightSlide.setTargetPosition(Slides_High);
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