package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MotorValuesConstants {

    @Config
    public static class SlidesConstants
    {
        public static final int START   = 0;
        public static final int GROUND  = -50;
        public static final int LOW     = 200;
        public static final int MEDIUM  = 950;
        public static final int HIGH    = 1600;
        public static final int STACK   = 75;

        public static final int SCORE_DROP = 300;

        public static final int Cone_1 = 50;
        public static final int Cone_2 = 150;
        public static final int Cone_3 = 250;
        public static final int Cone_4 = 350;

    }


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
        public static double START  = 0.83;
        public static double SCORE  = 0.05;
    }

    @Config
    public static class ArmLeftConstants
    {
        public static double START   = 0.17;
        public static double SCORE   = 0.95;
    }

    @Config
    public static class WristConstants
    {
        public static final double START     = 0.1;
        public static final double SCORE     = 0.78;
    }

    @Config
    public static class AlignmentConstants
    {
        public static final double UP = 0.23;
        public static final double SCORE = 0.58;
        public static final double DOWN = 0.7;
    }


    public static class IntakeServosConstants {
        public static final double MOVE_CONE = 0.5;
        public static final double INTAKE_LEFT_OUT = 0.92;
        public static final double INTAKE_RIGHT_OUT = 0.18;
        public static final double INTAKE_LEFT_IN = 0.5;
        public static final double INTAKE_RIGHT_IN = 0.59;
    }

    public static class GripperConstants {

        // Old values
        public static final double GRIPPER_GRAB = 0;
        public static final double GRIPPER_RELEASE = 0.25;

        public static final double RELEASE = 0.4;
        public static final double GRIP    = 0.53;
    }

}
