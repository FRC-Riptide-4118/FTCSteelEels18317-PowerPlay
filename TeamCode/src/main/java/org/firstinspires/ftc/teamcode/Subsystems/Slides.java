package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import static org.firstinspires.ftc.teamcode.Util.MotorValuesConstants.SlidesConstants;

@Config
public class Slides extends SubsystemBase {
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;

    PIDFCoefficients slidePIDF = new PIDFCoefficients(5, 0, 0, 0);


    public Slides(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_slide");

        // Slide motors
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setTargetPosition(SlidesConstants.start);
        rightSlide.setTargetPosition(SlidesConstants.start);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // PID Values
        leftSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, slidePIDF);
        rightSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, slidePIDF);

        setPower(1.0);
    }

    public void slidesToStart()
    {
        leftSlide.setTargetPosition(SlidesConstants.start);
        rightSlide.setTargetPosition(SlidesConstants.start);
    }

    public void slidesToLow()
    {
        leftSlide.setTargetPosition(SlidesConstants.low);
        rightSlide.setTargetPosition(SlidesConstants.low);
    }

    public void slidesToMedium()
    {
        leftSlide.setTargetPosition(SlidesConstants.medium);
        rightSlide.setTargetPosition(SlidesConstants.medium);
    }

    public void slidesToHigh()
    {
        leftSlide.setTargetPosition(SlidesConstants.high);
        rightSlide.setTargetPosition(SlidesConstants.high);
    }

    public void slidesUp()
    {
        leftSlide.setTargetPosition(leftSlide.getCurrentPosition()+300);
        rightSlide.setTargetPosition(rightSlide.getCurrentPosition()+300);
    }

    public void slidesToCone1()
    {
        leftSlide.setTargetPosition(SlidesConstants.cone1);
        rightSlide.setTargetPosition(SlidesConstants.cone1);
    }

    public void slidesToCone2()
    {
        leftSlide.setTargetPosition(SlidesConstants.cone2);
        rightSlide.setTargetPosition(SlidesConstants.cone2);
    }

    public void slidesToCone3()
    {
        leftSlide.setTargetPosition(SlidesConstants.cone3);
        rightSlide.setTargetPosition(SlidesConstants.cone3);
    }

    public void slidesToCone4()
    {
        leftSlide.setTargetPosition(SlidesConstants.cone4);
        rightSlide.setTargetPosition(SlidesConstants.cone4);
    }

    public void slidesToCone5()
    {
        leftSlide.setTargetPosition(SlidesConstants.cone5);
        rightSlide.setTargetPosition(SlidesConstants.cone5);
    }

    private void setPower(double power)
    {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    @Override
    public void periodic() {

    }
}