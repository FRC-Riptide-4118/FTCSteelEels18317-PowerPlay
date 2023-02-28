package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

public class MotorValuesConstants {

    //Slides Encoder Values
    @Config
    public static class SlidesConstants
    {
        public static int Start     = 0;
        public static int Ground    = -50;
        public static int Low       = 700;
        public static int Medium    = 1300;
        public static int High      = 1950;
    }

    // Encoder Values
    @Config
    public static class Arm1Constants
    {
        public static double Start 	 = 0.22;
		public static double Scoring = 0.85;
		public static double Front   = 0.85;
		public static double Cone1 	 = 0.03;
        public static double Cone2   = 0.06;
        public static double Cone3   = 0.1;
        public static double Cone4   = 0.12;
        public static double Cone5   = 0.2;
    }

    @Config
	public static class Arm2Constants
	{
		public static double Start   = 0.78;
        public static double Scoring = 0.15;
        public static double Front   = 0.15;
        public static double Cone1   = 0.97;
        public static double Cone2   = 0.94;
        public static double Cone3   = 0.9;
        public static double Cone4   = 0.88;
        public static double Cone5   = 0.8;
    }

    @Config
    public static class IntakeServosConstants {
        public static double Move_Cone         = 0.5;
        public static double IntakeLeft_out    = 0.9;
        public static double IntakeRight_out   = 0.9;
        public static double IntakeLeft_in     = 0.3;
        public static double IntakeRight_in    = 0.3;
    }

    @Config
    public static class GripperConstants {
        // Gripper Values
        public static double Gripper_Grab    = 0;
        public static double Gripper_Release = 0.25;
    }

}
