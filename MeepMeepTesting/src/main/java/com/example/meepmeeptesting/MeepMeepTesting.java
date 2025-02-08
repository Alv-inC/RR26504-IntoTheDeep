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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8.75, -62.75, Math.toRadians(90)))
                .waitSeconds(1)


//                .afterTime(0, new SequentialAction(
//                        new InstantAction(() -> lift.setTargetPosition(800)),
//                        new InstantAction(() -> externTele.lsecondary.setPosition(0.295)),
//                        new InstantAction(() -> externTele.rsecondary.setPosition(0.295)),
//                        new InstantAction(() -> externTele.primary.setPosition(0.2)),
//                        new InstantAction(() -> externTele.rotation.setPosition(0.47)),
//                        new InstantAction(() -> externTele.lext.setPosition(0.12)),
//                        new InstantAction(() -> externTele.rext.setPosition(0.12)),
//                        new SleepAction(1.4),
//                        chain.scoreSpecimen(),
//                        new SleepAction(1.2)
//                ))

                .strafeTo(new Vector2d(8.75, -38))

                .waitSeconds(1.7)



                .setReversed(true)



//increased all of these by +3
//                .afterTime(0.5, new SequentialAction(
//                        chain.startPosition()
//                ))
                .splineToConstantHeading(new Vector2d(31.5, -43), Math.toRadians(0))


                //first specimen
                .splineToConstantHeading(new Vector2d(39.5, -33), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(41, -21), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(51, -10), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(51, -61), Math.toRadians(270))
                .waitSeconds(0.6)

                //second specimen
                .splineToConstantHeading(new Vector2d(41, -28), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(43, -18), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(55, -15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(55, -57), Math.toRadians(270))
                .waitSeconds(0.4)


                //third specimen
//                .splineToConstantHeading(new Vector2d(57, -33), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(57, -21), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(64, -13), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(67, -59), Math.toRadians(270))
//                .waitSeconds(0.4)

                .splineToConstantHeading(new Vector2d(40, -55), Math.toRadians(270))
//                .afterTime(0.1, new SequentialAction(
//                        new InstantAction(() -> turret.setTargetPosition(-1250)),
//                        new SleepAction(1.5),
//                        chain.grabPosition()
//                ))
                .waitSeconds(2)
//                .afterTime(0,
//                        new SequentialAction(
//                                new InstantAction(() -> externTele.claw.setPosition(0.42)),
//                                new SleepAction(0.3),
//                                chain.scorePosition()
//                        ))
                .splineToConstantHeading(new Vector2d(8.75, -36), Math.toRadians(90))
//                .afterTime(0, new SequentialAction(
//                        new SleepAction(1.5),
//                        chain.scoreSpecimen()
//                ))
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