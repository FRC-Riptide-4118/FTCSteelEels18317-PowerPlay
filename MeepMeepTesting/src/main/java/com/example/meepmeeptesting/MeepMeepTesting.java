package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Stack;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose          = new Pose2d(37, 65, Math.toRadians(90));
        Pose2d closeHighJunction  = new Pose2d(31, 8, Math.toRadians(225));
        Pose2d closeMidJunction   = new Pose2d(29, 18, Math.toRadians(125));
        Pose2d Stack              = new Pose2d(67, 16, Math.toRadians(3));
        Pose2d Middle_Tile        = new Pose2d(40,16, Math.toRadians(3));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(46, 46, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(37, 65, Math.toRadians(90)))
                                .back(50)
                                //Pre-Load
                                .splineTo(new Vector2d(closeHighJunction.getX(), closeHighJunction.getY()), closeHighJunction.getHeading())
                                .waitSeconds(1)
                                //Stack 1
                                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                                .waitSeconds(0.5)
                                .setReversed(true)
                                .splineTo(new Vector2d(closeMidJunction.getX(), closeMidJunction.getY()), closeMidJunction.getHeading())
                                .waitSeconds(0.5)
                                .setReversed(false)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}