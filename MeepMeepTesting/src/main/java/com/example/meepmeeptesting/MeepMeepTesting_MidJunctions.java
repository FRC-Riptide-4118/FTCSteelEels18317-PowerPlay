package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_MidJunctions {

    public static class RightAutoConstants
    {
        public static Pose2d startPose           = new Pose2d(-37, 63, Math.toRadians(270));
        public static Pose2d preLoadMidJunction  = new Pose2d(-33, 16, Math.toRadians(45));
        public static Pose2d stack               = new Pose2d(-67.5, 7.5, Math.toRadians(180));
        public static Pose2d middleTile          = new Pose2d(-45, 10, Math.toRadians(180));
        public static Pose2d closeMidJunction    = new Pose2d(-34, 17, Math.toRadians(38));
    }


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(46, 46, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(RightAutoConstants.startPose)
                        .forward(20)
                        .lineToLinearHeading(new Pose2d(RightAutoConstants.preLoadMidJunction.getX(), RightAutoConstants.preLoadMidJunction.getY(), RightAutoConstants.preLoadMidJunction.getHeading()))
                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}