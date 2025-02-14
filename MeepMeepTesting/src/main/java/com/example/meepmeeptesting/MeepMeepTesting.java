package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.InstantAction;
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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-38, -62.75, Math.toRadians(90)))
                .waitSeconds(3)


//get ready to score & after time, score
// robot rotates to get new sample after 'scoring', may need to rotate turret to correct for incorrect angle

                .splineToLinearHeading(new Pose2d(-60, -50, Math.toRadians(65)), Math.toRadians(140))

                //SCORE
                //grab and score


                //grab
                .waitSeconds(3)
                //new score position
                .splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(90)), Math.toRadians(140))
                //SCORE

                //grab
                .waitSeconds(3)
                //new score position
                .splineToLinearHeading(new Pose2d(-56, -50, Math.toRadians(110)), Math.toRadians(140))
                .waitSeconds(3)
                //SCORE

                //grab

                //grab from summersible
                .splineToLinearHeading(new Pose2d(-25, -10, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(3)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(65)), Math.toRadians(180))




                .endTrajectory().build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}