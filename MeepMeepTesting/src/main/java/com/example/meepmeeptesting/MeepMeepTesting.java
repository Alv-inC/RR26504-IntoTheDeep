package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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
                //.setTangent(135)
                .strafeTo(new Vector2d(8.75, -38.5))
//                .afterTime(0.5, new SequentialAction(
//                        new InstantAction(() -> externTele.claw.setPosition(0.6)),
//                        new InstantAction(() -> externTele.lsecondary.setPosition(0.16)),
//                        new InstantAction(() -> externTele.rsecondary.setPosition(0.16)),
//                //wait
//                        new InstantAction(() -> externTele.primary.setPosition(0.25)),
//                        new InstantAction(() -> externTele.rotation.setPosition(0.47)),
//
//
//
//                        new InstantAction(() -> lift.setTargetPosition(820)),
//                        new InstantAction(() -> externTele.lext.setPosition(0.3)),
//                        new InstantAction(() -> externTele.rext.setPosition(0.3)),
//                        new SleepAction(1),
//                        new InstantAction(() -> externTele.lext.setPosition(0)),
//                        new InstantAction(() -> externTele.rext.setPosition(0))
//
//                        ))
                .setReversed(true)




                .splineToConstantHeading(new Vector2d(28, -44), Math.toRadians(0))


                .splineToConstantHeading(new Vector2d(36, -34), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40, -22), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(47, -7), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(47, -54), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(47, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(58, -7), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(58, -54), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(58, -34), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(59, -22), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(66, -7), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(66, -54), Math.toRadians(270))


                .splineToConstantHeading(new Vector2d(30, -62), Math.toRadians(270))
                //grab
                //turn 180
                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(8.75, -35.5), Math.toRadians(90))
                //turn 180;
                //score
                .waitSeconds(0.2)
                //end of first iteration

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30, -62), Math.toRadians(270))
                .waitSeconds(0.2)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(8.75, -35.5), Math.toRadians(90))
                .waitSeconds(0.2)

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30, -62), Math.toRadians(270))
                .waitSeconds(0.2)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(8.75, -35.5), Math.toRadians(90))
                .waitSeconds(0.2)

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30, -62), Math.toRadians(270))
                .waitSeconds(0.2)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(8.75, -35.5), Math.toRadians(90))
                .waitSeconds(0.2)

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30, -62), Math.toRadians(270))
                .waitSeconds(0.2)
                .setReversed(false)

                .endTrajectory().build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}