package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ServoConstants {
    //Slides Encoder Values
    public static int Slides_Start = 0;
    public static int Slides_Low = 300;
    public static int Slides_Medium = -790;
    public static int Slides_High = -1100;


    // Encoder Values
    public static double Arm1_Start = .9;
    public static double Arm2_Start = .1;
    public static double Arm1_Ground = 0;
    public static double Arm2_Ground = 1;
    public static double Arm1_Low = .2;
    public static double Arm2_Low = .2;
    public static double Arm1_Medium = 0;
    public static double Arm2_Medium = 0;
    public static double Arm1_High = .1;
    public static double Arm2_High = .1;

    // Gripper Values
    public static double Gripper_Grab = 0;
    public static double Gripper_Release = 0.4;
}
