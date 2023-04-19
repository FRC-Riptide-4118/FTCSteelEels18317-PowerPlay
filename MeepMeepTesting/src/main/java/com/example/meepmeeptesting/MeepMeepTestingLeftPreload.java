package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingLeftPreload {


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


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose           = new Pose2d(37, 63, Math.toRadians(90));
        Pose2d PreLoadMidJunction  = new Pose2d(31, 28, Math.toRadians(225));
        Pose2d Stack               = new Pose2d(65, 10, Math.toRadians(0));
        Pose2d Middle_Tile         = new Pose2d(40,18, Math.toRadians(0));
        Pose2d closeMidJunction    = new Pose2d(32, 20, Math.toRadians(130));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(46, 46, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(LeftAutoConstants.startPose)
                                .forward(55)
                                .setReversed(true)
                                .splineTo(new Vector2d(LeftAutoConstants.preLoadMidJunction.getX(), LeftAutoConstants.preLoadMidJunction.getY()), LeftAutoConstants.preLoadMidJunction.getHeading())
//                                .UNSTABLE_addTemporalMarkerOffset(0.1, intake::intakeServoOut)
//                                .UNSTABLE_addTemporalMarkerOffset(-3, gripper::gripCone)
//                                .UNSTABLE_addTemporalMarkerOffset(-1.75, slides::slidesToMedium)
//                                .UNSTABLE_addTemporalMarkerOffset(-1, arm::armScoring)
//                                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesDrop)
//                                .UNSTABLE_addTemporalMarkerOffset(0.3, gripper::releaseCone)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesUp)
//                                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToCone1)
//                                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToStack)
                                .waitSeconds(0.5)
                                //Stack 1
                .setReversed(false)
//                .splineTo(new Vector2d(LeftAutoConstants.stack.getX(), LeftAutoConstants.stack.getY()), LeftAutoConstants.stack.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, gripper::gripCone)
//                .UNSTABLE_addTemporalMarkerOffset(0.01, slides::slidesToMedium)
//                .setReversed(true)
//                .splineTo(new Vector2d(LeftAutoConstants.closeMidJunction.getX(), LeftAutoConstants.closeMidJunction.getY()), LeftAutoConstants.closeMidJunction.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.9, arm::armScoring)
//                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesDrop)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, gripper::releaseCone)
//                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesUp)
//                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToCone2)
//                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToGround)
//                .waitSeconds(0.25)
//                .setReversed(false)
//                //Stack 2
//                .splineTo(new Vector2d(LeftAutoConstants.stack.getX()+2.25, LeftAutoConstants.stack.getY()), LeftAutoConstants.stack.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, gripper::gripCone)
//                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesToMedium)
//                .setReversed(true)
//                .splineTo(new Vector2d(LeftAutoConstants.closeMidJunction.getX(), LeftAutoConstants.closeMidJunction.getY()), LeftAutoConstants.closeMidJunction.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.9, arm::armScoring)
//                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesDrop)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, gripper::releaseCone)
//                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesUp)
//                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToCone3)
//                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToGround)
//                .waitSeconds(0.25)
//                .setReversed(false)
//                //Stack 3
//                .splineTo(new Vector2d(LeftAutoConstants.stack.getX()+4, LeftAutoConstants.stack.getY()), LeftAutoConstants.stack.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, gripper::gripCone)
//                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesToMedium)
//                .setReversed(true)
//                .splineTo(new Vector2d(LeftAutoConstants.closeMidJunction.getX(), LeftAutoConstants.closeMidJunction.getY()), LeftAutoConstants.closeMidJunction.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.9, arm::armScoring)
//                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesDrop)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, gripper::releaseCone)
//                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesUp)
//                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToCone4)
//                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToGround)
//                .waitSeconds(0.25)
//                .setReversed(false)
//                //Stack 4
//                .splineTo(new Vector2d(LeftAutoConstants.stack.getX()+6, LeftAutoConstants.stack.getY()-0.5), LeftAutoConstants.stack.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.1, gripper::gripCone)
//                .UNSTABLE_addTemporalMarkerOffset(0.01, slides::slidesToMedium)
//                .setReversed(true)
//                .splineTo(new Vector2d(LeftAutoConstants.closeMidJunction.getX()+3.5, LeftAutoConstants.closeMidJunction.getY()+0.5), LeftAutoConstants.closeMidJunction.getHeading())
//                .UNSTABLE_addTemporalMarkerOffset(-0.9, arm::armScoring)
//                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesDrop)
//                .UNSTABLE_addTemporalMarkerOffset(0.3, gripper::releaseCone)
//                .UNSTABLE_addTemporalMarkerOffset(0.6, slides::slidesUp)
//                .UNSTABLE_addTemporalMarkerOffset(0.9, arm::armToStart)
//                .UNSTABLE_addTemporalMarkerOffset(1, slides::slidesToGround)
//                .waitSeconds(0.25)
//                .setReversed(false)
                                .splineTo(new Vector2d(LeftAutoConstants.middleTile.getX(), LeftAutoConstants.middleTile.getY()), LeftAutoConstants.middleTile.getHeading())
                                .waitSeconds(0.5)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}