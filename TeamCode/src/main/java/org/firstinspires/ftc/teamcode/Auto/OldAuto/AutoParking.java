package org.firstinspires.ftc.teamcode.Auto.OldAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "AutoParkingWithColorSensor")

public class AutoParking extends LinearOpMode {

  private ColorSensor colorSensor = null;
  private DcMotor  frontLeft  = null;
  private DcMotor  rearRight  = null;
  private DcMotor  frontRight  = null;
  private DcMotor  rearLeft  = null;
  private DcMotorEx  leftSlide  = null;
  private DcMotorEx  rightSlide  = null;
  private Servo  Gripper = null;
  private DcMotor arm = null;

  //Slides Encoder Values
  private static final int Slides_Start = 0;
  private static final int Slides_Low = -400;
  private static final int Slides_Medium = -900;
  private static final int Slides_High = -1100;

  //Arm Encoder Values
  private static final int Arm_Start = 0;
  private static final int Arm_Ground = -100;
  private static final int Arm_Low = 420;
  private static final int Arm_Medium = 420;
  private static final int Arm_High = 350;

  //Gripper Values
  private static final double Gripper_Release = 0.7;
  private static final double Gripper_Grab = 0;

  private boolean raisingToLow = false;
  private boolean returning = false;
  private boolean raisingToMiddle = false;
  private ElapsedTime armInTimer;

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
    Gripper = hardwareMap.get(Servo.class, "left_intake");
    colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
    arm = hardwareMap.get(DcMotor.class, "arm");
    leftSlide = hardwareMap.get(DcMotorEx.class, "left_slide");
    rightSlide = hardwareMap.get(DcMotorEx.class, "right_slide");

    frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

    Gripper.setPosition(Gripper_Grab);
    colorSensor.enableLed(true);

    HD_COUNTS_PER_REV = 537;
    DRIVE_GEAR_REDUCTION = 20;
    WHEEL_CIRCUMFERENCE_MM = 101.6 * Math.PI;
    DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;
    DRIVE_COUNTS_PER_IN = 2000/61;

    while (!isStarted()) {
      telemetry.addData("Red", colorSensor.red());
      telemetry.addData("Green", colorSensor.green());
      telemetry.addData("Blue", colorSensor.blue());
      telemetry.addData("green/red", ((double)colorSensor.green()/(double)colorSensor.red()));
      telemetry.addData("green/blue", ((double)colorSensor.green()/(double)colorSensor.blue()));
      telemetry.addData("red/blue", ((double)colorSensor.red()/(double)colorSensor.blue()));

      if (isRed(colorSensor.red(), colorSensor.green(), colorSensor.blue())) {
          telemetry.addLine("Left");
      }
      else if (isBlue(colorSensor.red(), colorSensor.green(), colorSensor.blue())) {
          telemetry.addLine("Middle");
      }
      else if (isGreen(colorSensor.red(), colorSensor.green(), colorSensor.blue())){
          telemetry.addLine("Right");
      }
      else { // Default case
          telemetry.addLine("Middle");
      }
      telemetry.update();
    }
    waitForStart();
    // Math for Traveling w/ Inches
    if (opModeIsActive()) {
      waitForStart();

      // Running Code
      leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      Reset_Encoders();
      drivetrain(24, 24, 24, 24, 0.2, 0.2, 0.2, telemetry);
      Reset_Encoders();

//      while (opModeIsActive()) {
//        telemetry.addData("Red", colorSensor.red());
//        telemetry.addData("Green", colorSensor.green());
//        telemetry.addData("Blue", colorSensor.blue());
//        telemetry.addData("green/red", ((double)colorSensor.green()/(double)colorSensor.red()));
//        telemetry.addData("green/blue", ((double)colorSensor.green()/(double)colorSensor.blue()));
//        telemetry.addData("red/blue", ((double)colorSensor.red()/(double)colorSensor.blue()));
//
//        if (isRed(colorSensor.red(), colorSensor.green(), colorSensor.blue())) {
//          telemetry.addLine("Left");
//        }
//        else if (isBlue(colorSensor.red(), colorSensor.green(), colorSensor.blue())) {
//          telemetry.addLine("Middle");
//        }
//        else if (isGreen(colorSensor.red(), colorSensor.green(), colorSensor.blue())){
//          telemetry.addLine("Right");
//        }
//        else { // Default case
//          telemetry.addLine("Middle");
//        }
//
//        telemetry.update();
//      }

      colorSensor.enableLed(true);  // Turn the LED off
      if (isRed(colorSensor.red(), colorSensor.green(), colorSensor.blue())) { // yellow
        // move to zone 1
        drivetrain(13, 13, 13, 13, 0.3, 0.3, 0.3,  telemetry);
        Reset_Encoders();
        drivetrain(-45, 45, 45, -45, 0.3, 0.3, 0.3, telemetry);
        Reset_Encoders();
        drivetrain(14, 14, 14, 14, 0.3, 0.3, 0.3,  telemetry);
        Reset_Encoders();
      } else if (isBlue(colorSensor.red(), colorSensor.green(), colorSensor.blue())) { // green
        // move to zone 2
        drivetrain(19, 19, 19, 19, 0.3, 0.3, 0.3,  telemetry);
        Reset_Encoders();
      } else if (isGreen(colorSensor.red(), colorSensor.green(), colorSensor.blue())){ // black
        // move to zone 3
        drivetrain(12, 12, 12, 12, 0.3, 0.3, 0.3,  telemetry);
        Reset_Encoders();
        drivetrain(47, 47, 47, 47, 0.3, -0.3, -0.3, telemetry);
        Reset_Encoders();
        drivetrain(17, 17, 17, 17, 0.3, 0.3, 0.3,  telemetry);
        Reset_Encoders();
      }
      else {
        // move to zone 2
        drivetrain(19, 19, 19, 19, 0.3, 0.3, 0.3,  telemetry);
        Reset_Encoders();
      }
    }
  }

  //Inches
  private void drivetrain(double frontLeftInches, double frontRightInches, double rearLeftInches, double rearRightInches, double Power, double frPower, double rlPower, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
    if (opModeIsActive()) {
//      frontRightTarget = frontRight.getCurrentPosition() + inchesToCounts(frontRightInches);
      rearRightTarget = rearRight.getCurrentPosition() + inchesToCounts(rearRightInches);
      frontLeftTarget = frontLeft.getCurrentPosition() + inchesToCounts(frontLeftInches);
//      rearLeftTarget = rearLeft.getCurrentPosition() + inchesToCounts(rearLeftInches);

//      frontRight.setTargetPosition(frontRightTarget);
      frontLeft.setTargetPosition(frontLeftTarget);
      rearRight.setTargetPosition(rearRightTarget);
//      rearLeft.setTargetPosition(rearLeftTarget);

      frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      frontLeft.setPower(Power);
      rearLeft.setPower(rlPower);
      rearRight.setPower(Power);
      frontRight.setPower(frPower);
      while (opModeIsActive() && (/*frontRight.isBusy()/* ||*/ frontLeft.isBusy()) && (rearRight.isBusy() /*|| rearLeft.isBusy()*/)) {
        // Color Sensor
        telemetry.addData("front left pos", frontLeft.getCurrentPosition());
        telemetry.addData("front right pos", frontRight.getCurrentPosition());
        telemetry.addData("back left pos", rearLeft.getCurrentPosition());
        telemetry.addData("back right pos", rearRight.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("front left power", frontLeft.getPower());
        telemetry.addData("front right power", frontRight.getPower());
        telemetry.addData("back left power", rearLeft.getPower());
        telemetry.addData("back right power", rearRight.getPower());
        telemetry.addLine();
        telemetry.addData("front left target", frontLeft.getTargetPosition());
        telemetry.addData("front right target", frontRight.getTargetPosition());
        telemetry.addData("back left target", rearLeft.getTargetPosition());
        telemetry.addData("back right target", rearRight.getTargetPosition());
        telemetry.update();
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

  // InchesToCounts
  public int inchesToCounts(double inches) {
    return (int) (inches * DRIVE_COUNTS_PER_IN);
  }

  // Red Color Sensor
  public boolean isRed(int red, int green, int blue) {
    return (red > green &&
            red > blue);
  }

  // Green Color Sensor
  public boolean isGreen(int red, int green, int blue) {
    return (green > red &&
            green > blue);
  }

  // Blue Color Sensor
  public boolean isBlue(int red, int green, int blue) {
    return (blue > red &&
            blue > green);
  }


}

