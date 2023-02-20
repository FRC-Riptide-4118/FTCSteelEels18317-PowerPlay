package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingMid {
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
                        drive.trajectorySequenceBuilder(new Pose2d(37, 65, Math.toRadians(90)))
                                .back(25)
                                .splineTo(new Vector2d(PreLoadMidJunction.getX(), PreLoadMidJunction.getY()), PreLoadMidJunction.getHeading())
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> {})
                                .splineTo(new Vector2d(Middle_Tile.getX(), Middle_Tile.getY()), Middle_Tile.getHeading())
                                .addDisplacementMarker(() -> {})
                                //Stack 1
                                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.95)
                                .setReversed(true)
                                .splineTo(new Vector2d(closeMidJunction.getX(), closeMidJunction.getY()), closeMidJunction.getHeading())
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.25)
                                .setReversed(false)
                                //Stack 2
                                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.95)
                                .setReversed(true)
                                .splineTo(new Vector2d(closeMidJunction.getX(), closeMidJunction.getY()), closeMidJunction.getHeading())
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.25)
                                .setReversed(false)
                                //Stack 3
                                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.95)
                                .setReversed(true)
                                .splineTo(new Vector2d(closeMidJunction.getX(), closeMidJunction.getY()), closeMidJunction.getHeading())
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.25)
                                .setReversed(false)
                                //Stack 4
                                .splineTo(new Vector2d(Stack.getX(), Stack.getY()), Stack.getHeading())
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.95)
                                .setReversed(true)
                                .splineTo(new Vector2d(closeMidJunction.getX(), closeMidJunction.getY()), closeMidJunction.getHeading())
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.25)
                                .setReversed(false)
                                .splineTo(new Vector2d(Middle_Tile.getX(), Middle_Tile.getY()), Middle_Tile.getHeading())
                                .addDisplacementMarker(() -> {})
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}