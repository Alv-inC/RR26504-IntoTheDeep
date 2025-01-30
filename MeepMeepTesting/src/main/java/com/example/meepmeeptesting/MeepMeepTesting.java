package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8.75, -62.75, Math.toRadians(90)))
                .waitSeconds(3)
                .strafeTo(new Vector2d(8.75, -37))

                .waitSeconds(3)



                .setReversed(true)



//increased all of these by +3
                .splineToConstantHeading(new Vector2d(31.5, -44), Math.toRadians(0))

                //first specimen
                .splineToConstantHeading(new Vector2d(41.5, -33), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(45.5, -21), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(51.5, -13), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(51.5, -61), Math.toRadians(270))
                .waitSeconds(0.3)

                //second specimen
                .splineToConstantHeading(new Vector2d(50, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(54, -13), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(54, -61), Math.toRadians(270))
                .waitSeconds(0.3)

//
//                //third specimen
//                .splineToConstantHeading(new Vector2d(52.5, -33), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(52.5, -21), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(55, -5), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(58.5, -61), Math.toRadians(270))
//                .waitSeconds(0.3)



//
//
//                .splineToConstantHeading(new Vector2d(30, -62), Math.toRadians(270))
//                //grab
//                //turn 180
//                .waitSeconds(0.2)
//                .splineToConstantHeading(new Vector2d(8.75, -35.5), Math.toRadians(90))
//                //turn 180;
//                //score
//                .waitSeconds(0.2)
//                //end of first iteration
//
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(30, -62), Math.toRadians(270))
//                .waitSeconds(0.2)
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(8.75, -35.5), Math.toRadians(90))
//                .waitSeconds(0.2)
//
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(30, -62), Math.toRadians(270))
//                .waitSeconds(0.2)
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(8.75, -35.5), Math.toRadians(90))
//                .waitSeconds(0.2)
//
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(30, -62), Math.toRadians(270))
//                .waitSeconds(0.2)
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(8.75, -35.5), Math.toRadians(90))
//                .waitSeconds(0.2)
//
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(30, -62), Math.toRadians(270))
//                .waitSeconds(0.2)
//                .setReversed(false)

                .endTrajectory().build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}