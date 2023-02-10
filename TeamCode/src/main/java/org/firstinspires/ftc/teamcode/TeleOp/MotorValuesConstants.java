package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MotorValuesConstants {

    //Slides Encoder Values
    public static int Slides_Start = 0;
    public static int Slides_Ground = 5;
    public static int TeleOpSlides_Ground = 10;
    public static int Slides_Low = 700;
    public static int Slides_Medium = 1300;
    public static int Slides_High = 1950;

    // Encoder Values
    public static double Arm1_Start = 0.2;
    public static double Arm2_Start = 0.8;
    public static double Arm1_Ground = 0.2;
    public static double Arm2_Ground = 0.8;
    public static double Arm1_Low = 0.85;
    public static double Arm2_Low = 0.15;
    public static double Arm1_Medium = 0.85;
    public static double Arm2_Medium = 0.15;
    public static double Arm1_High = 0.85;
    public static double Arm2_High = 0.15;
    public static double Arm1_Scoring = 0.85;
    public static double Arm2_Scoring = 0.15;
    public static double Arm1_Front = 0.85;
    public static double Arm2_Front = 0.15;
    public static double Arm1_Cone1 = 0;
    public static double Arm2_Cone1 = 1;
    public static double Move_Cone = 0.5;
    public static double Intake1_out = 0.9;
    public static double Intake2_out = 0.28;
    public static double Intake1_in = 0.3;
    public static double Intake2_in = 0.75;
    // .2 .8 is GROUND/START
    // .85 and .15 for SCORING



    // Gripper Values
    public static double Gripper_Grab = 0;
    public static double Gripper_Release = 0.25;
}
