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

@Autonomous(name = "basket auto")
public class basketAuto extends LinearOpMode {
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
        if (isStopRequested()) return;

        TrajectoryActionBuilder trajectory = drive.actionBuilder(startPose)
                .waitSeconds(1)

                .afterTime(0, new SequentialAction(
                        new InstantAction(() -> lift.setTargetPosition(800)),
                        new InstantAction(() -> externTele.lsecondary.setPosition(0.295)),
                        new InstantAction(() -> externTele.rsecondary.setPosition(0.295)),
                        new InstantAction(() -> externTele.primary.setPosition(0.2)),
                        new InstantAction(() -> externTele.rotation.setPosition(0.47)),
                        new InstantAction(() -> externTele.lext.setPosition(0.12)),
                        new InstantAction(() -> externTele.rext.setPosition(0.12)),
                        new SleepAction(1.4),
                        chain.scoreSpecimen(),
                        new SleepAction(1.2)
                ))

                .strafeTo(new Vector2d(8.75, -38))

                .waitSeconds(1.7)

                .setReversed(true)

                .afterTime(0.5, new SequentialAction(
                        chain.startPosition()
                ))
//baskets starts
                .strafeTo(new Vector2d(8.75, -52))
                .waitSeconds(1.7)

                .strafeTo(new Vector2d(56, -52))
                .waitSeconds(1.7)
                .afterTime(1,
                        new SequentialAction(
                                chain.grabSample()
                        ))

                .strafeTo(new Vector2d(-52, -52))
                .waitSeconds(1.7)
                .afterTime(1,
                        new SequentialAction(
                                chain.scoreLowBasket()
                        ))

                .endTrajectory();
        Action trajectoryAction = trajectory.build();

        Actions.runBlocking(new ParallelAction(
                trajectoryAction,
                lift.updateAction(),
                turret.updateAction()
        ));
    }
    }