package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MotorValuesConstants {

    //Slides Encoder Values
    public static class SlidesConstants
    {
        public static int Start     = 0;
        public static int Ground    = -10;
        public static int Low       = 700;
        public static int Medium    = 1300;
        public static int High      = 1950;
    }

    // Encoder Values
    public static class Arm1
    {
        public static double Start 	= 0.15;
		public static double Ground = 0.2;
		public static double Low 	= 0.85;
		public static double Medium = 0.85;
		public static double High 	= 0.85;
		public static double Scoring= 0.8;
		public static double Front 	= 0.85;
		public static double Cone1 	= 0;
    }
	public static class Arm2
	{
		public static double Start  = 0.85;
        public static double Ground = 0.8;
        public static double Low    = 0.15;
        public static double Medium = 0.15;
        public static double High   = 0.15;
        public static double Scoring= 0.2;
        public static double Front  = 0.15;
        public static double Cone1  = 1;
	}

    public static class IntakeServos {
        public static double Move_Cone = 0.5;
        public static double IntakeLeft_out = 0.9;
        public static double IntakeRight_out = 0.9;
        public static double IntakeLeft_in = 0.3;
        public static double IntakeRight_in = 0.3;
    }

    // Gripper Values
    public static double Gripper_Grab = 0;
    public static double Gripper_Release = 0.25;
}
