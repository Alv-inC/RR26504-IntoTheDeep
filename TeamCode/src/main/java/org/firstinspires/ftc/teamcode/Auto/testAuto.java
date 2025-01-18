package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ChainActions;
import org.firstinspires.ftc.teamcode.Subsystems.IDK;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@Autonomous(name = "Testing Auto")
public class testAuto extends LinearOpMode {
//    private Lift lift = new Lift(hardwareMap);
//    private ChainActions chain = new ChainActions(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(8.75, -62.75, 90));
        Pose2d startPose = new Pose2d(8.75, -62.75, Math.toRadians(90));


        waitForStart();
        if(isStopRequested()) return;

        TrajectoryActionBuilder trajectory = drive.actionBuilder(startPose)
//                .afterTime(0, lift.setTargetPositionAction(500))
                .waitSeconds(0.2)
                //.setTangent(135)
                .strafeTo(new Vector2d(8.75, -35.5))



                .setReversed(true)
                .splineToConstantHeading(new Vector2d(25, -44), Math.toRadians(0))


                .splineToConstantHeading(new Vector2d(36, -34), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40, -22), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(47, -12), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(47, -54), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(47, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(58, -12), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(58, -54), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(58, -34), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(59, -22), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(66, -12), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(66, -54), Math.toRadians(270))


                .splineToConstantHeading(new Vector2d(33, -62), Math.toRadians(270))
                //grab
                //turn 180
                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(8.75, -35.5), Math.toRadians(90))
                //turn 180;
                //score
                .waitSeconds(0.2)
                //end of first iteration

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(33, -62), Math.toRadians(270))
                .waitSeconds(0.2)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(8.75, -35.5), Math.toRadians(90))
                .waitSeconds(0.2)

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(33, -62), Math.toRadians(270))
                .waitSeconds(0.2)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(8.75, -35.5), Math.toRadians(90))
                .waitSeconds(0.2)

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(33, -62), Math.toRadians(270))
                .waitSeconds(0.2)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(8.75, -35.5), Math.toRadians(90))
                .waitSeconds(0.2)

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(33, -62), Math.toRadians(270))
                .waitSeconds(0.2)
                .setReversed(false)

                .endTrajectory();


        Actions.runBlocking(new IDK.RaceParallelCommand(
                trajectory.build()
//                lift.updateAction()
        ));

        Action trajectoryAction = trajectory.build();



    }
}
