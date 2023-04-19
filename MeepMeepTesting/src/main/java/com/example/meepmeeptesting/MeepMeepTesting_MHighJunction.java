package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_MHighJunction {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose           = new Pose2d(-37, 63, Math.toRadians(270));
        Pose2d preLoadMidJunction  = new Pose2d(-33, 16, Math.toRadians(45));
        Pose2d MidJunction         = new Pose2d(-35, 14, Math.toRadians(52));
        Pose2d stack               = new Pose2d(-67.5, 7.5, Math.toRadians(180));
        Pose2d middleTile          = new Pose2d(-45, 10, Math.toRadians(180));
        Pose2d closeMidJunction    = new Pose2d(-34, 17, Math.toRadians(38));

        Pose2d parkLeft           = new Pose2d(-64, 11.9, Math.toRadians(0));
        Pose2d parkMiddle         = new Pose2d(-43, 11.9, Math.toRadians(0));
        Pose2d parkRight          = new Pose2d(-30, 11.9, Math.toRadians(180));

        double inchesToStack = 24.5;


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(46, 46, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .forward(55)
//                                .UNSTABLE_addTemporalMarkerOffset(-12, arm::armToMiddle)
//                                .UNSTABLE_addTemporalMarkerOffset(0, slides::slidesToMedium)
//                                .UNSTABLE_addTemporalMarkerOffset(0, wrist::toScoring)
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, alignment::score)
//                                .UNSTABLE_addTemporalMarkerOffset(0, arm::armToPreScore)
//                .UNSTABLE_addTemporalMarkerOffset(-2, intake::close)
//                .UNSTABLE_addTemporalMarkerOffset(0, intake::open)
                                .setReversed(true)
                                .splineTo(new Vector2d(MidJunction.getX()+1, MidJunction.getY()), MidJunction.getHeading())
                                // PreLoad
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, arm::armToScore)
//                                .UNSTABLE_addTemporalMarkerOffset(0.2, gripper::releaseCone)
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::armToStack)
//                                .UNSTABLE_addTemporalMarkerOffset(0.7, slides::slidesToCone1)
                                // Cone 1
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(stack.getX()+23, stack.getY(), stack.getHeading()))
                                .forward(inchesToStack)
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, gripper::gripCone)
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, alignment::down)
//                                .UNSTABLE_addTemporalMarkerOffset(0.02, intake::out)
//                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, slides::slidesToMedium)
//                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, arm::armToPreScore)
//                                .waitSeconds(wristIntakeArmDelay)
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, wrist::toScoring)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, intake::off)
//                                .UNSTABLE_addTemporalMarkerOffset(1.2, alignment::score)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(MidJunction.getX(), MidJunction.getY(), MidJunction.getHeading() + Math.PI))
//
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, arm::armToScore)
//                                .UNSTABLE_addTemporalMarkerOffset(0.2, gripper::releaseCone)
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::armToStack)
//                                .UNSTABLE_addTemporalMarkerOffset(0.7, slides::slidesToCone2)
                                // Cone 2
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(stack.getX()+23, stack.getY(), stack.getHeading()))
                                .forward(inchesToStack)
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, gripper::gripCone)
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, alignment::down)
//                                .UNSTABLE_addTemporalMarkerOffset(0.02, intake::out)
//                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, slides::slidesToMedium)
//                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, arm::armToPreScore)
//                                .waitSeconds(wristIntakeArmDelay)
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, wrist::toScoring)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, intake::off)
//                                .UNSTABLE_addTemporalMarkerOffset(1.2, alignment::score)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(MidJunction.getX(), MidJunction.getY(), MidJunction.getHeading() + Math.PI))
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, arm::armToScore)
//                                .UNSTABLE_addTemporalMarkerOffset(0.2, gripper::releaseCone)
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::armToStack)
//                                .UNSTABLE_addTemporalMarkerOffset(0.7, slides::slidesToCone3)
                                // Cone 3
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(stack.getX()+23, stack.getY(), stack.getHeading()))
                                .forward(inchesToStack)
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, gripper::gripCone)
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, alignment::down)
//                                .UNSTABLE_addTemporalMarkerOffset(0.02, intake::out)
//                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, slides::slidesToMedium)
//                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, arm::armToPreScore)
//                                .waitSeconds(wristIntakeArmDelay)
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, wrist::toScoring)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, intake::off)
//                                .UNSTABLE_addTemporalMarkerOffset(1.2, alignment::score)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(MidJunction.getX(), MidJunction.getY(), MidJunction.getHeading() + Math.PI))
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, arm::armToScore)
//                                .UNSTABLE_addTemporalMarkerOffset(0.2, gripper::releaseCone)
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::armToStack)
//                                .UNSTABLE_addTemporalMarkerOffset(0.7, slides::slidesToCone4)
                                // Cone 4
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(stack.getX()+23, stack.getY(), stack.getHeading()))
                                .forward(inchesToStack)
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, gripper::gripCone)
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, alignment::down)
//                                .UNSTABLE_addTemporalMarkerOffset(0.02, intake::out)
//                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, slides::slidesToMedium)
//                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, arm::armToPreScore)
//                                .waitSeconds(wristIntakeArmDelay)
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, wrist::toScoring)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, intake::off)
//                                .UNSTABLE_addTemporalMarkerOffset(1.2, alignment::score)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(MidJunction.getX(), MidJunction.getY(), MidJunction.getHeading() + Math.PI))
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, arm::armToScore)
//                                .UNSTABLE_addTemporalMarkerOffset(0.2, gripper::releaseCone)
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::armToStart)
//                                .UNSTABLE_addTemporalMarkerOffset(0.7, slides::slidesToCone5)
                                // Cone 5??
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(stack.getX()+23, stack.getY(), stack.getHeading()))
                                .forward(inchesToStack)
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, gripper::gripCone)
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, alignment::down)
//                                .UNSTABLE_addTemporalMarkerOffset(-0.1, intake::in)
//                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, slides::slidesToMedium)
//                                .UNSTABLE_addTemporalMarkerOffset(slidesArmDelay, arm::armToPreScore)
//                                .waitSeconds(wristIntakeArmDelay)
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, wrist::toScoring)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, intake::off)
//                                .UNSTABLE_addTemporalMarkerOffset(1.2, alignment::score)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(MidJunction.getX(), MidJunction.getY(), MidJunction.getHeading() + Math.PI))
//                                .UNSTABLE_addTemporalMarkerOffset(0.0, arm::armToScore)
//                                .UNSTABLE_addTemporalMarkerOffset(0.2, gripper::releaseCone)
//                                .UNSTABLE_addTemporalMarkerOffset(0.5, slides::slidesUp)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, wrist::toStart)
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, arm::armToStack)
//                                .UNSTABLE_addTemporalMarkerOffset(0.7, slides::slidesToCone5)
//                                .UNSTABLE_addTemporalMarkerOffset(0.9, alignment::up)
                                // PARK PARK PARK
                                .setReversed(false)
                                .lineToLinearHeading(parkMiddle)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}