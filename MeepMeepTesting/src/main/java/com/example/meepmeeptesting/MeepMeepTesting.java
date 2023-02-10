package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(46, 46, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(37, 65, Math.toRadians(90)))

                                .setReversed(true)
//                                .back(5)
                                .splineToSplineHeading(new Pose2d(36, 14, Math.toRadians(50)), Math.toRadians(90))

//                                .setReversed(true)
//                                .back(26)
//                                .splineTo(new Vector2d(36, 12), Math.toRadians(230))
                                .addDisplacementMarker(() -> {})
                                .back(5)
                                .waitSeconds(0.5)

                                .setReversed(false)
                                .splineTo(new Vector2d(55, 15), Math.toRadians(0))
                                .addDisplacementMarker(() -> {})
                                .forward(5)
                                .waitSeconds(0.5)

                                .setReversed(true)
                                .back(5)
                                .splineTo(new Vector2d(33, 10), Math.toRadians(225))
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.5)
                                //Scores cone from stack

                                //Keep cycling...

                                //Setup for parking
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(35, 13, Math.toRadians(0)), Math.toRadians(225))
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