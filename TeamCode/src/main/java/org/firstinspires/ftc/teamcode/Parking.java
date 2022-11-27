package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "parkingWithColorSensor")
@Disabled
public class Parking extends LinearOpMode {

  private ColorSensor = null;
  private DcMotor  frontLeft  = null;
  private DcMotor  rearRight  = null;
  private DcMotor  frontRight  = null;
  private DcMotor  rearLeft  = null;
  private DcMotor  leftSlide  = null;
  private DcMotor  rightSlide  = null;

  int frontRightTarget;
  int frontLeftTarget;
  int rearRightTarget;
  int rearLeftTarget;
  double DRIVE_COUNTS_PER_IN;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int HD_COUNTS_PER_REV;
    int DRIVE_GEAR_REDUCTION;
    double WHEEL_CIRCUMFERENCE_MM;
    double DRIVE_COUNTS_PER_MM;

    frontLeft = hardwareMap.get(DcMotor.class, "front_left_wheel");
    rearLeft = hardwareMap.get(DcMotor.class, "rear_left_wheel");
    frontRight = hardwareMap.get(DcMotor.class, "front_right_wheel");
    rearRight = hardwareMap.get(DcMotor.class, "rear_right_wheel");
    ColorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

    frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    rearRight.setDirection(DcMotorSimple.Direction.FORWARD);

    HD_COUNTS_PER_REV = 537;
    DRIVE_GEAR_REDUCTION = 20;
    WHEEL_CIRCUMFERENCE_MM = 101.6 * Math.PI;
    DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    waitForStart();
    // Math for Traveling w/ Inches
    if (opModeIsActive()) {
      waitForStart();
      // Running Code
      leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      Reset_Encoders();
      drivetrain(5, 5, 5, 5, .5);
      Reset_Encoders();
      colorSensor.enableLed(true);  // Turn the LED on
      if (colorSensor.red() >= 5) {
        // move to zone 1
        colorSensor.enableLed(false);  // Turn the LED off
        drivetrain(-7, 7, -7, 7, 0.3);
        Reset_Encoders();
        drivetrain(-7, -7, -7, -7, 0.3);
      } else if (colorSensor.red() <= 2) {
        // move to zone 2
        colorSensor.enableLed(false);  // Turn the LED off
        drivetrain(7, 7, 7, 7, 0.3);
        Reset_Encoders();
        drivetrain(-7, -7, -7, -7,0.3);
      } else {
        colorSensor.enableLed(false);  // Turn the LED off
        drivetrain(7, 7, 7, 7,0.3);
        // move to zone 3
        Reset_Encoders();
        drivetrain(-7, -7, -7, -7,0.3);
      }
      Reset_Encoders();
      drivetrain(6, -6, 6, -6,0.4);
      Reset_Encoders();
    }
  }

  //Inches
  private void drivetrain(double frontLeftInches, double frontRightInches, double rearLeftInches, double rearRightInches, double Power) {
    if (opModeIsActive()) {
      frontRightTarget = (int) (frontRight.getCurrentPosition() + frontRightInches * DRIVE_COUNTS_PER_IN);
      rearRightTarget = (int) (rearRight.getCurrentPosition() + rearRightInches * DRIVE_COUNTS_PER_IN);
      frontLeftTarget = (int) (frontLeft.getCurrentPosition() + frontLeftInches * DRIVE_COUNTS_PER_IN);
      rearLeftTarget = (int) (rearLeft.getCurrentPosition() + rearLeftInches * DRIVE_COUNTS_PER_IN);
      frontRight.setTargetPosition(frontRightTarget);
      frontLeft.setTargetPosition(frontLeftTarget);
      rearRight.setTargetPosition(rearRightTarget);
      rearLeft.setTargetPosition(rearLeftTarget);
      frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontLeft.setPower(Power);
      rearLeft.setPower(Power);
      rearRight.setPower(Power);
      frontRight.setPower(Power);
      while (opModeIsActive() && (frontRight.isBusy() || frontLeft.isBusy()) && (rearRight.isBusy() || rearLeft.isBusy())) {
      }
      frontRight.setPower(0);
      frontLeft.setPower(0);
      rearRight.setPower(0);
      rearLeft.setPower(0);
    }
  }

  //Resetting Encoders
  private void Reset_Encoders() {
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftTarget = 0;
    frontRightTarget = 0;
    rearRightTarget = 0;
    rearLeftTarget = 0;
  }
}
