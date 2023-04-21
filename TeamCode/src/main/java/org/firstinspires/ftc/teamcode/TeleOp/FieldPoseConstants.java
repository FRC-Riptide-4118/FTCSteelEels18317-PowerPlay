package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class FieldPoseConstants {

    @Config
    public static class LeftAutoConstants
    {
        public static Pose2d startPose           = new Pose2d(37, 63, Math.toRadians(270));
        public static Pose2d tileToStack         = new Pose2d(45, 13, Math.toRadians(0));
        public static Pose2d stack               = new Pose2d(68, 12.1, Math.toRadians(0));
        public static Pose2d middleTile          = new Pose2d(50, 10, Math.toRadians(0));
        public static Pose2d MidJunction         = new Pose2d(35, 14, Math.toRadians(128));
        public static Pose2d parkLeft            = new Pose2d(67, 10.9, Math.toRadians(0));
        public static Pose2d parkMiddle          = new Pose2d(43, 11.9, Math.toRadians(0));
        public static Pose2d parkRight           = new Pose2d(19, 11.9, Math.toRadians(180));

        public static Vector2d allyStartTile    = new Vector2d(-35, 60);
        public static Pose2d allyScoreCone      = new Pose2d(-32, 35, Math.toRadians(135));
        public static Pose2d parkAllyLeftSafe   = new Pose2d(-12, 35, Math.toRadians(90));
        public static Pose2d parkAllyRightSafe  = new Pose2d(-60, 35, Math.toRadians(90));
        public static Pose2d parkAllyMiddleSafe = new Pose2d(-36, 35, Math.toRadians(90));

        //Not in Use
        public static Pose2d closeMidJunction   = new Pose2d(33.25, 16.75, Math.toRadians(135));
    }

    @Config
    public static class RightAutoConstants
    {
        public static Pose2d startPose           = new Pose2d(-37,   63, Math.toRadians(270));
        public static Pose2d MidJunction         = new Pose2d(-35,   14, Math.toRadians(52));
        public static Pose2d stack               = new Pose2d(-67.5, 7.5, Math.toRadians(180));
        public static Pose2d middleTile          = new Pose2d(-45,   10, Math.toRadians(180));
        public static Pose2d closeMidJunction    = new Pose2d(-34,   17, Math.toRadians(38));
        public static Pose2d SafeHigh            = new Pose2d(0,     0, Math.toRadians(52));
        public static Pose2d SafeHighTile        = new Pose2d(-9,    30, Math.toRadians(135));


        public static Pose2d parkLeft           = new Pose2d(-64, 11.9, Math.toRadians(0));
        public static Pose2d parkMiddle         = new Pose2d(-43, 11.9, Math.toRadians(0));
        public static Pose2d parkRight          = new Pose2d(-30, 11.9, Math.toRadians(180));

        public static Pose2d parkSafeLeft       = new Pose2d(-64, 35, Math.toRadians(0));
        public static Pose2d parkSafeMiddle     = new Pose2d(-43, 35, Math.toRadians(0));
        public static Pose2d parkSafeRight      = new Pose2d(-30, 35, Math.toRadians(180));

        public static Pose2d preLoadMidJunction  = new Pose2d(-33, 16, Math.toRadians(45));
    }

}
