package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MotorValuesConstants {

    @Config
    public static class SlidesConstants
    {
        public static int START   = 0;
        public static int GROUND  = -50;
        public static int LOW     = 150;
        public static int MEDIUM  = 800;
        public static int HIGH    = 1450;
        public static int STACK   = 75;

        public static final int SCORE_DROP = 300;

        public static int Cone_1 = 300;
        public static int Cone_2 = 180;
        public static int Cone_3 = 50;
        public static int Cone_4 = 50;
        public static int Cone_5 = 0;

    }


    @Deprecated
    public static class Arm1Constants
    {
        public static final double START = 0.2;
		public static final double SCORING = 0.85;
		public static final double FRONT = 0.85;
		public static final double CONE_1 = 0.03;
        public static final double CONE_2 = 0.06;
        public static final double CONE_3 = 0.1;
        public static final double CONE_4 = 0.12;
        public static final double CONE_5 = 0.2;
    }

    @Deprecated
	public static class Arm2Constants
	{
		public static final double START = 0.8;
        public static final double SCORING = 0.15;
        public static final double FRONT = 0.15;
        public static final double CONE_1 = 0.97;
        public static final double CONE_2 = 0.94;
        public static final double CONE_3 = 0.9;
        public static final double CONE_4 = 0.88;
        public static final double CONE_5 = 0.8;
    }

    @Config
    public static class ArmRightConstants
    {
        public static double START      = 0.18;
        public static double CONE_FLIP  = 0.23;
        public static double OFF_GROUND = 0.23;
        public static double STACK      = 0.20;
        public static double MIDDLE     = 0.50;
        public static double PRE_SCORE  = 0.85;
        public static double SCORE      = 1.00;
    }

    @Config
    public static class ArmLeftConstants
    {
        public static double START      = 0.18;
        public static double CONE_FLIP  = 0.23;
        public static double OFF_GROUND = 0.23;
        public static double STACK      = 0.20;
        public static double MIDDLE     = 0.50;
        public static double PRE_SCORE  = 0.85;
        public static double SCORE      = 1.00;
    }

    @Config
    public static class WristConstants
    {
        public static double START     = 0.1;
        public static double SCORE     = 0.78;
    }

    @Config
    public static class AlignmentConstants
    {
        public static double UP = 0.23;
        public static double SCORE = 0.58;
        public static double DOWN = 0.7;
    }


    @Config
    public static class IntakeServosConstants {
        public static double MOVE_CONE = 0.5;
        public static double INTAKE_LEFT_OUT = 0.9;
        public static double INTAKE_RIGHT_OUT = 0.25;
        public static double INTAKE_LEFT_IN = 0.5;
        public static double INTAKE_RIGHT_IN = 0.6;
    }

    public static class GripperConstants {

        // Old values
        public static final double OLD_GRIPPER_GRAB = 0;
        public static final double OLD_GRIPPER_RELEASE = 0.25;

        public static double RELEASE = 0.4;
        public static double GRIP    = 0.53;
    }

}
