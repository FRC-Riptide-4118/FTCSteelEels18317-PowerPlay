package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

public class MotorValuesConstants {

    //Slides Encoder Values
    @Config
    public static class SlidesConstants
    {
        public static final int START   = 0;
        public static final int GROUND  = -50;
        public static final int LOW     = 700;
        public static final int MEDIUM  = 1300;
        public static final int HIGH    = 1950;
        public static final int STACK   = 75;
    }

    // Encoder Values
    @Config
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

    @Config
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
    public static class IntakeServosConstants {
        public static final double MOVE_CONE = 0.5;
        public static final double INTAKE_LEFT_OUT = 0.92;
        public static final double INTAKE_RIGHT_OUT = 0.18;
        public static final double INTAKE_LEFT_IN = 0.5;
        public static final double INTAKE_RIGHT_IN = 0.59;
    }

    @Config
    public static class GripperConstants {
        // Gripper Values
        public static final double GRIPPER_GRAB = 0;
        public static final double GRIPPER_RELEASE = 0.25;
    }

}
