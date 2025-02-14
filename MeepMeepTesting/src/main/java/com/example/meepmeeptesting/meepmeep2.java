package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmeep2 {
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

                .strafeTo(new Vector2d(8.75, -52))
                .waitSeconds(1.7)

                .strafeTo(new Vector2d(56, -52))
                .waitSeconds(1.7)

                .strafeTo(new Vector2d(-52, -52))
                .waitSeconds(1.7)

                .endTrajectory().build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}