package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class FieldPoseConstants {

    public static class LeftAutoConstants
    {
        public static Pose2d startPose           = new Pose2d(37, 63, Math.toRadians(90));
        public static Pose2d preLoadMidJunction  = new Pose2d(33, 18, Math.toRadians(135));
        public static Pose2d stack               = new Pose2d(68, 12, Math.toRadians(0));
        public static Pose2d middleTile         = new Pose2d(40, 11, Math.toRadians(0));
        public static Pose2d closeMidJunction    = new Pose2d(33, 18, Math.toRadians(128));
    }





}
