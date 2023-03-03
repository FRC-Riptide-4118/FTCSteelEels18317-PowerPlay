package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class FieldPoseConstants {

    public static class LeftAutoConstants
    {
        public static Pose2d startPose           = new Pose2d(37, 63, Math.toRadians(270));
        public static Pose2d preLoadMidJunction  = new Pose2d(33, 16, Math.toRadians(135));
        public static Pose2d stack               = new Pose2d(66.5, 9.5, Math.toRadians(0));
        public static Pose2d middleTile          = new Pose2d(45, 10, Math.toRadians(0));
        public static Pose2d closeMidJunction    = new Pose2d(34, 17, Math.toRadians(128));
    }

    public static class RightAutoConstants
    {
        public static Pose2d startPose           = new Pose2d(-37, 63, Math.toRadians(270));
        public static Pose2d preLoadMidJunction  = new Pose2d(-33, 16, Math.toRadians(45));
        public static Pose2d stack               = new Pose2d(-66.5, 7.5, Math.toRadians(180));
        public static Pose2d middleTile          = new Pose2d(-45, 10, Math.toRadians(180));
        public static Pose2d closeMidJunction    = new Pose2d(-34, 17, Math.toRadians(38));
    }

}
