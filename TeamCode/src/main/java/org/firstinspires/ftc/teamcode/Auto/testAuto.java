package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ChainActions;
import org.firstinspires.ftc.teamcode.Subsystems.IDK;
import org.firstinspires.ftc.teamcode.Subsystems.InitializeTeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Autonomous(name = "Testing Auto")
public class testAuto extends LinearOpMode {
private Lift lift;
private Turret turret;
// private ChainActions chain = new ChainActions(hardwareMap);
private InitializeTeleOp externTele;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(8.75, -62.75, Math.toRadians(90)));
        Pose2d startPose = new Pose2d(8.75, -62.75, Math.toRadians(90));
        externTele = new InitializeTeleOp();
        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);
        externTele.initialize(hardwareMap, telemetry);
        externTele.lext.setPosition(0.05);
        externTele.rext.setPosition(0.05);
        externTele.claw.setPosition(0.75);
        turret.setTargetPosition(0);
        lift.setTargetPosition(30);


        waitForStart();
        if(isStopRequested()) return;

        TrajectoryActionBuilder trajectory = drive.actionBuilder(startPose)
//                .afterTime(0, lift.setTargetPositionAction(500))
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




                .splineToConstantHeading(new Vector2d(25, -44), Math.toRadians(0))


                .splineToConstantHeading(new Vector2d(33, -34), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(37, -22), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -7), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(44, -54), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(44, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(55, -7), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(55, -54), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(55, -34), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(56, -22), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(63, -7), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(63, -54), Math.toRadians(270))


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

                .endTrajectory();



        Action trajectoryAction = trajectory.build();

        Actions.runBlocking(new ParallelAction(
                trajectoryAction,
                lift.updateAction(),
                turret .updateAction()
        ));
//        Actions.runBlocking(new IDK.RaceParallelCommand(
//                trajectoryAction,
//                lift.updateAction(),
//                turret .updateAction()
//        ));


    }
}
