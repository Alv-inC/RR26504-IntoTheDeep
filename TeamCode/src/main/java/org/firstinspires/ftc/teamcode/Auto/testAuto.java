package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ChainActions;
import org.firstinspires.ftc.teamcode.Subsystems.IDK;
import org.firstinspires.ftc.teamcode.Subsystems.InitializeTeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import java.util.Arrays;

@Autonomous(name = "Testing Auto")
public class testAuto extends LinearOpMode {
private Lift lift;
private Turret turret;
// private ChainActions chain = new ChainActions(hardwareMap);
private InitializeTeleOp externTele;
private ChainActions chain;

VelConstraint slower = new MinVelConstraint(Arrays.asList(
        new TranslationalVelConstraint(30),
        new AngularVelConstraint(Math.PI / 2)
));

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(8.75, -62.75, Math.toRadians(90)));
        Pose2d startPose = new Pose2d(8.75, -62.75, Math.toRadians(90));
        externTele = new InitializeTeleOp();
        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);
        chain = new ChainActions(hardwareMap);
        externTele.initialize(hardwareMap);
        externTele.lext.setPosition(0);
        externTele.rext.setPosition(0);
        externTele.claw.setPosition(0.42);
        turret.setTargetPosition(0);
        lift.setTargetPosition(30);



        waitForStart();
        if(isStopRequested()) return;

        TrajectoryActionBuilder trajectory = drive.actionBuilder(startPose)
                .waitSeconds(1)


                .afterTime(0, new SequentialAction(
                        chain.scorePosition(),
                        new SleepAction(1.4),
                        chain.scoreSpecimen(),
                        new SleepAction(1.2)
                ))

                .strafeTo(new Vector2d(8.75, -36))

                .waitSeconds(2.2)



                .setReversed(true)



//increased all of these by +3
                .afterTime(0.5, new SequentialAction(
                        chain.startPosition()
                ))
                .splineToConstantHeading(new Vector2d(31.5, -43), Math.toRadians(0))


                //first specimen
                .splineToConstantHeading(new Vector2d(44.5, -33), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46, -21), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(56, -10), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(56, -61), Math.toRadians(270))
                .waitSeconds(0.6)

                //second specimen
                .splineToConstantHeading(new Vector2d(45, -28), Math.toRadians(90), slower)
                .splineToConstantHeading(new Vector2d(48, -20), Math.toRadians(90), slower)
                .splineToConstantHeading(new Vector2d(58, -17), Math.toRadians(270), slower)
                .splineToConstantHeading(new Vector2d(58, -57), Math.toRadians(270), slower)
                .waitSeconds(0.4)


                //third specimen
//                .splineToConstantHeading(new Vector2d(57, -33), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(57, -21), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(64, -13), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(67, -59), Math.toRadians(270))
//                .waitSeconds(0.4)

                .splineToConstantHeading(new Vector2d(45, -55), Math.toRadians(270))
                .turn(Math.toRadians(180))
                .afterTime(0.1, new SequentialAction(
                    chain.grabPosition()
                ))
                .waitSeconds(2)
                .afterTime(0,
                        new SequentialAction(
                        new InstantAction(() -> externTele.claw.setPosition(0.42)),
                        chain.scorePosition()
                ))
                .splineToConstantHeading(new Vector2d(8.75, -36), Math.toRadians(90))
                .turn(Math.toRadians(180))
                .waitSeconds(1.5)
                .afterTime(0, new SequentialAction(
                        chain.scoreSpecimen()
                ))
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
