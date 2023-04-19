package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MotorValuesConstants {

    @Config
    public static class SlidesConstants
    {
        public static int start = 0;
        public static int low = 150;
        public static int medium = 800;
        public static int high = 1450;

        public static int cone1 = 300;
        public static int cone2 = 180;
        public static int cone3 = 50;
        public static int cone4 = 50;
        public static int cone5 = 0;
    }

    @Config
    public static class ArmConstants
    {
        public static double start = 0.18;
        public static double coneFlip = 0.23;
        public static double stack = 0.20;
        public static double middle = 0.50;
        public static double preScore = 0.85;
        public static double score = 1.00;
    }

    @Config
    public static class WristConstants
    {
        public static double start = 0.1;
        public static double score = 0.78;
    }

    @Config
    public static class AlignmentConstants
    {
        public static double up = 0.23;
        public static double score = 0.58;
        public static double down = 0.7;
    }

    @Config
    public static class IntakeServosConstants {
        public static double moveCone = 0.5;
        public static double leftOut = 0.9;
        public static double rightOut = 0.25;
        public static double leftIn = 0.5;
        public static double rightIn = 0.6;
    }

    @Config
    public static class GripperConstants {
        public static double release = 0.4;
        public static double grip = 0.53;
    }

}
