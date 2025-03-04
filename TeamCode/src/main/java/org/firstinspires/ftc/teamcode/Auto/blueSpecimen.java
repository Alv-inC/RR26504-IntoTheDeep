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

@Autonomous(name = "RED AUTO")
public class blueSpecimen extends LinearOpMode {
    private Lift lift;
    private Turret turret;
    // private ChainActions chain = new ChainActions(hardwareMap);
    private InitializeTeleOp externTele;
    private ChainActions chain;

    VelConstraint slower = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(30),
            new AngularVelConstraint(Math.PI / 2)
    ));

    VelConstraint faster = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(70),
            new AngularVelConstraint(Math.PI)
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
        externTele.claw.setPosition(0.58);
        externTele.rotation.setPosition(1);
        turret.setTargetPosition(0);
        lift.setTargetPosition(30);



        waitForStart();
        if(isStopRequested()) return;

        TrajectoryActionBuilder trajectory = drive.actionBuilder(startPose)


                .afterTime(0, new SequentialAction(
                        new InstantAction(() -> externTele.primary.setPosition(0.65)),
                        new InstantAction(() -> externTele.lext.setPosition(0.15)),
                        new InstantAction(() -> externTele.rext.setPosition(0.15)),
                        new SleepAction(0.3),
                        new InstantAction(() -> externTele.lext.setPosition(0)),
                        new InstantAction(() -> externTele.rext.setPosition(0)),
                        new InstantAction(() -> externTele.lsecondary.setPosition(0.38)),
                        new InstantAction(() -> externTele.rsecondary.setPosition(0.38)),
                        new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                        new InstantAction(() -> lift.setTargetPosition(945)),
                        new SleepAction(1.4),
                        chain.scoreSpecimen()
                ))

                .strafeTo(new Vector2d(8.75, -34))

                .waitSeconds(1.3)



                .setReversed(true)



//increased all of these by +3
                .afterTime(0.5, new SequentialAction(
                        chain.startPosition(false)
                ))
                .splineToConstantHeading(new Vector2d(31.5, -43), Math.toRadians(0))


                //first specimen
                //first specimen
                .splineToConstantHeading(new Vector2d(36, -33), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(37, -21), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46, -7), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(46, -46), Math.toRadians(270))
                .afterTime(0, new SequentialAction(
                        new InstantAction(() -> externTele.lext.setPosition(0.05)),
                        new InstantAction(() -> externTele.rext.setPosition(0.05)),
                        new InstantAction(() -> externTele.primary.setPosition(0.34)),
                        new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                        new InstantAction(() -> externTele.lsecondary.setPosition(0.41)),
                        new InstantAction(() -> externTele.rsecondary.setPosition(0.41)),
                        new SleepAction(0.3),
                        new InstantAction(() -> turret.setTargetPosition(-1250))
                ))

                //second specimen
                .splineToConstantHeading(new Vector2d(42, -28), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(43, -18), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(55, -3), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(55, -52.5), Math.toRadians(270))



                //third specimen
//                .splineToConstantHeading(new Vector2d(57, -33), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(57, -21), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(64, -13), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(67, -59), Math.toRadians(270))
//                .waitSeconds(0.4)

                .splineToConstantHeading(new Vector2d(55, -40), Math.toRadians(270))
                .afterTime(0, new SequentialAction(
                        new InstantAction(() -> lift.setTargetPosition(0)),
                        new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                        new InstantAction(() -> externTele.lsecondary.setPosition(0.155)),
                        new InstantAction(() -> externTele.rsecondary.setPosition(0.155)),
                        new InstantAction(() -> externTele.primary.setPosition(0.6))

                ))


                .splineToConstantHeading(new Vector2d(55, -57.5), Math.toRadians(270))
                .afterTime(0,
                        new SequentialAction(
                                new InstantAction(() -> externTele.claw.setPosition(0.58)),
                                new SleepAction(0.3),
                                chain.scorePositionAuto()
                        ))
                .waitSeconds(1.3)
                //.splineToConstantHeading(new Vector2d(4, -35), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(2, -37, Math.toRadians(110)), Math.toRadians(90))
                .afterTime(0.2, new SequentialAction(
                        chain.scoreSpecimen(),
                        new InstantAction(() -> externTele.lext.setPosition(0.05)),
                        new InstantAction(() -> externTele.rext.setPosition(0.05)),
                        new InstantAction(() -> externTele.primary.setPosition(0.34)),
                        new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                        new InstantAction(() -> externTele.lsecondary.setPosition(0.41)),
                        new InstantAction(() -> externTele.rsecondary.setPosition(0.41)),
                        new SleepAction(0.3),
                        new InstantAction(() -> turret.setTargetPosition(-1250))
                ))

                //3rd
                .splineToConstantHeading(new Vector2d(2, -43), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(49, -45), Math.toRadians(270))
                .afterTime(0, new SequentialAction(
                        new InstantAction(() -> lift.setTargetPosition(0)),
                        new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                        new InstantAction(() -> externTele.lsecondary.setPosition(0.155)),
                        new InstantAction(() -> externTele.rsecondary.setPosition(0.155)),
                        new InstantAction(() -> externTele.primary.setPosition(0.6))

                ))
                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(49, -57), Math.toRadians(270))
                .waitSeconds(1)
                .afterTime(0,
                        new SequentialAction(
                                new InstantAction(() -> externTele.claw.setPosition(0.58)),
                                new SleepAction(0.3),
                                chain.scorePositionAuto()
                        ))
                .waitSeconds(1.3)
                //.splineToConstantHeading(new Vector2d(4, -35), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, -37, Math.toRadians(110)), Math.toRadians(90))
                .afterTime(0.3, new SequentialAction(
                        chain.scoreSpecimen(),
                        new InstantAction(() -> externTele.lext.setPosition(0.05)),
                        new InstantAction(() -> externTele.rext.setPosition(0.05)),
                        new InstantAction(() -> externTele.primary.setPosition(0.34)),
                        new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                        new InstantAction(() -> externTele.lsecondary.setPosition(0.41)),
                        new InstantAction(() -> externTele.rsecondary.setPosition(0.41)),
                        new SleepAction(0.3),
                        new InstantAction(() -> turret.setTargetPosition(-1250))
                ))


//
//
//
////                //third score
//                .splineToConstantHeading(new Vector2d(43, -45), Math.toRadians(270), slower)
//                .afterTime(1.5, new SequentialAction(
//                        chain.grabPositionAuto()
//                ))
//                .waitSeconds(3.5)
//                .splineToConstantHeading(new Vector2d(43, -56.5), Math.toRadians(270), slower)
//                .afterTime(1.5,
//                        new SequentialAction(
//                                new InstantAction(() -> externTele.claw.setPosition(0.42)),
//                                chain.scorePositionAuto()
//                        ))
//                .waitSeconds(4)
//                .splineToConstantHeading(new Vector2d(7.7, -40.3), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(7.7, -38), Math.toRadians(90), slower)
//                .afterTime(2, new SequentialAction(
//                        chain.scoreSpecimen()
//                ))
//
//                //third score
//                .splineToConstantHeading(new Vector2d(43, -45), Math.toRadians(270), slower)
//                .afterTime(1.5, new SequentialAction(
//                        chain.grabPositionAuto()
//                ))
//                .waitSeconds(3.5)
//                .splineToConstantHeading(new Vector2d(43, -56.5), Math.toRadians(270), slower)
//                .afterTime(1.5,
//                        new SequentialAction(
//                                new InstantAction(() -> externTele.claw.setPosition(0.42)),
//                                chain.scorePositionAuto()
//                        ))
//                .waitSeconds(4)
//                .splineToConstantHeading(new Vector2d(7.7, -40.3), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(7.7, -38), Math.toRadians(90), slower)
//                .afterTime(2, new SequentialAction(
//                        chain.scoreSpecimen()
//                ))
//
//                //fourth score
//                .splineToConstantHeading(new Vector2d(43, -45), Math.toRadians(270), slower)
//                .afterTime(1.5, new SequentialAction(
//                        chain.grabPositionAuto()
//                ))
//                .waitSeconds(3.5)
//                .splineToConstantHeading(new Vector2d(43, -56.5), Math.toRadians(270), slower)
//                .afterTime(1.5,
//                        new SequentialAction(
//                                new InstantAction(() -> externTele.claw.setPosition(0.42)),
//                                chain.scorePositionAuto()
//                        ))
//                .waitSeconds(4)
//                .splineToConstantHeading(new Vector2d(7.7, -40.3), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(7.7, -38), Math.toRadians(90), slower)
//                .afterTime(2, new SequentialAction(
//                        chain.scoreSpecimen()
//                ))

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
