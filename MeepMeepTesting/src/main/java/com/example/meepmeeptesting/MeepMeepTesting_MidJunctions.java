package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_MidJunctions {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(46, 46, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(37, 65, Math.toRadians(270)))
                                //Goes to Mid_Junction
                                .splineToLinearHeading(new Pose2d(36, 14, Math.toRadians(315)), Math.toRadians(270))
                                .addDisplacementMarker(() -> {})
                                .back(3)
                                .waitSeconds(0.5)
                                //Scores pre-loaded cone

                                //Goes to stack
                                .splineTo(new Vector2d(50, 13), Math.toRadians(0))
                                .forward(10)
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.5)
                                //Pickup cone

                                //Goes to Mid_Junction
                                .setReversed(true)
                                .splineTo(new Vector2d(34, 16), Math.toRadians(-220))
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.5)
                                //Score cone from stack

                                // Runs back to the stack
                                .setReversed(false)
                                .splineTo(new Vector2d(50, 13), Math.toRadians(0))
                                .forward(10)
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.5)
                                //Start cycling

                                //Goes to Mid_Junction
                                .setReversed(true)
                                .splineTo(new Vector2d(34, 16), Math.toRadians(-220))
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.5)
                                //Score another cone from stack

                                // Runs back to the stack
                                .setReversed(false)
                                .splineTo(new Vector2d(50, 13), Math.toRadians(0))
                                .forward(10)
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.5)
                                //Start cycling


                                .setReversed(true)
                                .back(10)
                                .splineTo(new Vector2d(33, 9), Math.toRadians(225))
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.5)

                                //Keep cycling...

                                //Setup for parking
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(36, 12, Math.toRadians(0)), Math.toRadians(-220))
                                .addDisplacementMarker(() -> {})
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