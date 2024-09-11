package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepField {
    static int position;
    static Pose2d myPose;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        myPose = new Pose2d(-36, -36, Math.toRadians(0));
        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36/*12*/, /*58*/-60, Math.toRadians(90/*-90*/)))
                                //far blue through middle

                                //tape #1
                                    /*.lineToLinearHeading(new Pose2d(-36,34,Math.toRadians(180)))
                                    //place pixel on tape
                                    .lineToLinearHeading(new Pose2d(-36,10,Math.toRadians(0)))
                                    .forward(72)*/
                                //tape #2
                                    /*.lineToLinearHeading(new Pose2d(-36,34,Math.toRadians(270)))
                                    //place pixel on tape
                                    .lineToLinearHeading(new Pose2d(-60,34, Math.toRadians(270)))
                                    .lineToLinearHeading(new Pose2d(-60,10))
                                    .forward(96)*/
                                //tape #3
                                    /*.lineToLinearHeading(new Pose2d(-36,34,Math.toRadians(0)))
                                    //place pixel on tape
                                    .lineToLinearHeading(new Pose2d(-36,10,Math.toRadians(0)))
                                    .forward(72)*/

                                /*.lineToLinearHeading(new Pose2d(36,34,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(36,10,Math.toRadians(180)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(-60,34,Math.toRadians(180)))

                                .lineToLinearHeading(new Pose2d(-60,10,Math.toRadians(0)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(36,34,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(36,10,Math.toRadians(180)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(-60,34,Math.toRadians(180)))
                                .build()*/



                                //far blue through side
                                //tape #1
                                    //.lineToLinearHeading(new Pose2d(-36,34,Math.toRadians(180)))
                                    //place pixel on tape
                                //tape #2
                                    //.lineToLinearHeading(new Pose2d(-36,34,Math.toRadians(270)))
                                    //place pixel on tape
                                //tape #3
                                    //.lineToLinearHeading(new Pose2d(-36,34,Math.toRadians(0)))
                                    //place pixel on tape

                                //pick up 2 white pixels

                                /*.lineToLinearHeading(new Pose2d(-36,58,Math.toRadians(0)))
                                .forward(72)
                                .lineToLinearHeading(new Pose2d(36,34,Math.toRadians(0)))
                                //place preloaded pixel on board

                                .lineToLinearHeading(new Pose2d(36,58,Math.toRadians(180)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(-60,34,Math.toRadians(180)))

                                .lineToLinearHeading(new Pose2d(-60,58,Math.toRadians(0)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(36,34,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(36,58,Math.toRadians(180)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(-60,34,Math.toRadians(180)))
                                .build()*/



                                //close blue down middle
                                //tape #1
                                    /*.lineToLinearHeading(new Pose2d(12, 34, Math.toRadians(180)))
                                    .lineToLinearHeading(new Pose2d(36, 34, Math.toRadians(0)))*/
                                //tape #2
                                    /*.lineToLinearHeading(new Pose2d(12, 34, Math.toRadians(-90)))
                                    .lineToLinearHeading(new Pose2d(36, 34, Math.toRadians(0)))*/
                                //tape #3
                                    /*.lineToLinearHeading(new Pose2d(36, 58, Math.toRadians(180)))
                                    .lineToLinearHeading(new Pose2d(36, 34,Math.toRadians(180)))
                                .turn(Math.toRadians(180))*/

                                /*.lineToLinearHeading(new Pose2d(36,10,Math.toRadians(180)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(36, 10, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(36, 34,Math.toRadians(0)))

                                .lineToLinearHeading(new Pose2d(36,10,Math.toRadians(180)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(36, 10, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(36,34,Math.toRadians(0)))
                                .build()*/



                                //close blue down side
                                //tape #1
                                    /*.lineToLinearHeading(new Pose2d(12, 34, Math.toRadians(180)))
                                    .lineToLinearHeading(new Pose2d(36, 34, Math.toRadians(0)))*/
                                //tape #2
                                    /*.lineToLinearHeading(new Pose2d(12, 34, Math.toRadians(-90)))
                                    .lineToLinearHeading(new Pose2d(36, 34, Math.toRadians(0)))*/
                                //tape #3
                                    /*.lineToLinearHeading(new Pose2d(36, 58, Math.toRadians(180)))
                                    .lineToLinearHeading(new Pose2d(36, 34,Math.toRadians(180)))
                                    .turn(Math.toRadians(180))*/

                                /*.lineToLinearHeading(new Pose2d(36,58,Math.toRadians(180)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(-60,34,Math.toRadians(180)))

                                .lineToLinearHeading(new Pose2d(-60,58,Math.toRadians(0)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(36,34,Math.toRadians(0)))

                                .lineToLinearHeading(new Pose2d(36,58,Math.toRadians(180)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(-60,34,Math.toRadians(180)))

                                .lineToLinearHeading(new Pose2d(-60,58,Math.toRadians(0)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(36,34,Math.toRadians(0)))
                                .build()*/









                                //far red through middle

                                //tape #1
                                    /*.lineToLinearHeading(new Pose2d(-36,-36,Math.toRadians(180)))
                                    //place pixel on tape
                                    .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(0)))
                                    .forward(72)*/
                                //tape #2
                                    .lineToLinearHeading(new Pose2d(-36,-36,Math.toRadians(90)))
                                    //place pixel on tape
                                    .lineToLinearHeading(new Pose2d(-60,-36, Math.toRadians(90)))
                                    .lineToLinearHeading(new Pose2d(-60,-12))
                                    .forward(96)
                                //tape #3
                                    /*.lineToLinearHeading(new Pose2d(-36,-36,Math.toRadians(0)))
                                    //place pixel on tape
                                    .lineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(0)))
                                    .forward(72)*/

                                .lineToLinearHeading(new Pose2d(36,-36,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(36,-12,Math.toRadians(180)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(-60,-36,Math.toRadians(180)))

                                .lineToLinearHeading(new Pose2d(-60,-12,Math.toRadians(0)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(36,-36,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(36,-12,Math.toRadians(180)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(-60,-36,Math.toRadians(180)))
                                .build()



                                //far red through side
                                //tape #1
                                    //.lineToLinearHeading(new Pose2d(-36,-36,Math.toRadians(180)))
                                    //place pixel on tape
                                //tape #2
                                    //.lineToLinearHeading(new Pose2d(-36,-36,Math.toRadians(90)))
                                    //place pixel on tape
                                //tape #3
                                    //.lineToLinearHeading(new Pose2d(-36,-36,Math.toRadians(0)))
                                    //place pixel on tape

                                //pick up 2 white pixels

                                /*.lineToLinearHeading(new Pose2d(-36,-60,Math.toRadians(0)))
                                .forward(72)
                                .lineToLinearHeading(new Pose2d(36,-36,Math.toRadians(0)))
                                //place preloaded pixel on board

                                .lineToLinearHeading(new Pose2d(36,-60,Math.toRadians(180)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(-60,-36,Math.toRadians(180)))
                                //pick up 2 white pixels

                                .lineToLinearHeading(new Pose2d(-60,-60,Math.toRadians(0)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(36,-36,Math.toRadians(0)))
                                //place white pixels on board

                                .lineToLinearHeading(new Pose2d(36,-60,Math.toRadians(180)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(-60,-36,Math.toRadians(180)))
                                //pick up 2 white pixels
                                .build()*/



                                //close red down middle
                                //tape #1
                                    /*.lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(180)))
                                    .lineToLinearHeading(new Pose2d(36, -36, Math.toRadians(0)))*/
                                    //place pixel on tape
                                //tape #2
                                    /*.lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(90)))
                                    .lineToLinearHeading(new Pose2d(36, -36, Math.toRadians(0)))*/
                                    //place pixel on tape
                                //tape #3
                                    /*.lineToLinearHeading(new Pose2d(36, -60, Math.toRadians(180)))
                                    .lineToLinearHeading(new Pose2d(36, -36,Math.toRadians(180)))
                                    //place pixel on tape
                                    .turn(Math.toRadians(180))*/

                                //place pre-loaded pixel on board

                                /*.lineToLinearHeading(new Pose2d(36,-12,Math.toRadians(180)))
                                .forward(96)
                                //pick up 2 white pixels

                                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(36, -36,Math.toRadians(0)))
                                //place white pixels on board

                                .lineToLinearHeading(new Pose2d(36,-12,Math.toRadians(180)))
                                .forward(96)
                                //pick up 2 white pixels

                                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(36, -36,Math.toRadians(0)))
                                //place white pixels on board
                                .build()*/



                                //close red down side
                                //tape #1
                                    /*.lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(180)))
                                    .lineToLinearHeading(new Pose2d(36, -36, Math.toRadians(0)))*/
                                    //place pixel on tape
                                //tape #2
                                    /*.lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(90)))
                                    .lineToLinearHeading(new Pose2d(36, -36, Math.toRadians(0)))*/
                                    //place pixel on tape
                                //tape #3
                                    /*.lineToLinearHeading(new Pose2d(36, -60, Math.toRadians(180)))
                                    .lineToLinearHeading(new Pose2d(36, -36,Math.toRadians(180)))
                                    //place pixel on tape
                                    .turn(Math.toRadians(180))*/

                                /*.lineToLinearHeading(new Pose2d(36,-60,Math.toRadians(180)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(-60,-36,Math.toRadians(180)))
                                //pick up 2 white pixels

                                .lineToLinearHeading(new Pose2d(-60,-60,Math.toRadians(0)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(36,-36,Math.toRadians(0)))
                                //place white pixels on board

                                .lineToLinearHeading(new Pose2d(36,-60,Math.toRadians(180)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(-60,-36,Math.toRadians(180)))
                                //pick up 2 white pixels

                                .lineToLinearHeading(new Pose2d(-60,-60,Math.toRadians(0)))
                                .forward(96)
                                .lineToLinearHeading(new Pose2d(36,-36,Math.toRadians(0)))
                                //place white pixels on board
                                .build()*/



                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }
}

//this is just the defualt meep meep testing code, change later
//also change the powerplay field to the center stage one once its out