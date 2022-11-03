package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "ACTIVELeftBLUERandomizerWarehouse (Blocks to Java)")
@Disabled
public class ACTIVELeftBLUERandomizerWarehouse extends LinearOpMode {

  private DcMotor rightFrontAsDcMotor;
  private DcMotor rightRearAsDcMotor;
  private DcMotor ArmAsDcMotor;
  private DistanceSensor left_distancesensorAsDistanceSensor;
  private DistanceSensor right_distancesensorAsDistanceSensor;
  private DcMotor intakeMotorAsDcMotor;
  private DcMotor leftFrontAsDcMotor;
  private DcMotor leftRearAsDcMotor;

  int rightTarget;
  int leftTarget;
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

    rightFrontAsDcMotor = hardwareMap.get(DcMotor.class, "rightFrontAsDcMotor");
    rightRearAsDcMotor = hardwareMap.get(DcMotor.class, "rightRearAsDcMotor");
    ArmAsDcMotor = hardwareMap.get(DcMotor.class, "ArmAsDcMotor");
    left_distancesensorAsDistanceSensor = hardwareMap.get(DistanceSensor.class, "left_distancesensorAsDistanceSensor");
    right_distancesensorAsDistanceSensor = hardwareMap.get(DistanceSensor.class, "right_distancesensorAsDistanceSensor");
    intakeMotorAsDcMotor = hardwareMap.get(DcMotor.class, "intakeMotorAsDcMotor");
    leftFrontAsDcMotor = hardwareMap.get(DcMotor.class, "leftFrontAsDcMotor");
    leftRearAsDcMotor = hardwareMap.get(DcMotor.class, "leftRearAsDcMotor");

    waitForStart();
    // Math for Traveling w/ Inches
    if (opModeIsActive()) {
      HD_COUNTS_PER_REV = 28;
      DRIVE_GEAR_REDUCTION = 20;
      WHEEL_CIRCUMFERENCE_MM = 100 * Math.PI;
      DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
      DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;
      waitForStart();
      // Reverse Motors and Color Sensor
      rightFrontAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      rightRearAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      // Running Code
      ArmAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      Reset_Encoders();
      do_something(14.8, 14.8, 0.2);
      Reset_Encoders();
      if (left_distancesensorAsDistanceSensor.getDistance(DistanceUnit.CM) <= 10) {
        // Raise arm to level 2
        do_something(7, 7, 0.3);
        ArmAsDcMotor.setTargetPosition(2500);
        ArmAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmAsDcMotor.setPower(1);
        Reset_Encoders();
        do_something(-7, -7, 0.3);
      } else if (right_distancesensorAsDistanceSensor.getDistance(DistanceUnit.CM) <= 10) {
        // Raise arm to level 3
        do_something(7, 7, 0.3);
        ArmAsDcMotor.setTargetPosition(3900);
        ArmAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmAsDcMotor.setPower(1);
        Reset_Encoders();
        do_something(-7, -7, 0.3);
      } else {
        do_something(7, 7, 0.3);
        // Raise arm to level 1
        ArmAsDcMotor.setTargetPosition(1350);
        ArmAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmAsDcMotor.setPower(1);
        Reset_Encoders();
        do_something(-7, -7, 0.3);
      }
      Reset_Encoders();
      do_something(6, -6, 0.4);
      Reset_Encoders();
      do_something(7, 7, 0.4);
      Reset_Encoders();
      intakeMotorAsDcMotor.setPower(1);
      sleep(2500);
      intakeMotorAsDcMotor.setPower(0);
      do_something(-5, -5, 0.5);
      ArmAsDcMotor.setTargetPosition(1350);
      ArmAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      ArmAsDcMotor.setPower(1);
      Reset_Encoders();
      do_something(-18, 18, 0.4);
      Reset_Encoders();
      do_something(38, 38, 0.7);
      ArmAsDcMotor.setTargetPosition(0);
      ArmAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      ArmAsDcMotor.setPower(1);
      Reset_Encoders();
      do_something(10, 10, 0.3);
      Reset_Encoders();
    }
  }

  //Inches
  private void do_something(double leftInches, double rightInches, double Power) {
    if (opModeIsActive()) {
      rightTarget = (int) (rightFrontAsDcMotor.getCurrentPosition() + rightInches * DRIVE_COUNTS_PER_IN);
      rightTarget = (int) (rightRearAsDcMotor.getCurrentPosition() + rightInches * DRIVE_COUNTS_PER_IN);
      leftTarget = (int) (leftFrontAsDcMotor.getCurrentPosition() + leftInches * DRIVE_COUNTS_PER_IN);
      leftTarget = (int) (leftRearAsDcMotor.getCurrentPosition() + leftInches * DRIVE_COUNTS_PER_IN);
      rightFrontAsDcMotor.setTargetPosition(rightTarget);
      leftFrontAsDcMotor.setTargetPosition(leftTarget);
      rightRearAsDcMotor.setTargetPosition(rightTarget);
      leftRearAsDcMotor.setTargetPosition(leftTarget);
      rightFrontAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      leftRearAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rightRearAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      leftFrontAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rightFrontAsDcMotor.setPower(Power);
      leftRearAsDcMotor.setPower(Power);
      rightRearAsDcMotor.setPower(Power);
      leftFrontAsDcMotor.setPower(Power);
      while (opModeIsActive() && (rightFrontAsDcMotor.isBusy() || leftFrontAsDcMotor.isBusy()) && (rightRearAsDcMotor.isBusy() || leftRearAsDcMotor.isBusy())) {
      }
      rightFrontAsDcMotor.setPower(0);
      leftFrontAsDcMotor.setPower(0);
      rightRearAsDcMotor.setPower(0);
      leftRearAsDcMotor.setPower(0);
    }
  }

  //Resetting Encoders
  private void Reset_Encoders() {
    rightFrontAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightRearAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftFrontAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftRearAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftTarget = 0;
    rightTarget = 0;
  }
}
