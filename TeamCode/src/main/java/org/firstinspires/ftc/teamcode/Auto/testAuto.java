package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
private ChainActions chain;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(8.75, -62.75, Math.toRadians(90)));
        Pose2d startPose = new Pose2d(8.75, -62.75, Math.toRadians(90));
        externTele = new InitializeTeleOp();
        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);
        chain = new ChainActions(hardwareMap);
        externTele.initialize(hardwareMap);
        externTele.lext.setPosition(0.05);
        externTele.rext.setPosition(0.05);
        externTele.claw.setPosition(0.62);
        turret.setTargetPosition(0);
        lift.setTargetPosition(30);



        waitForStart();
        if(isStopRequested()) return;

        TrajectoryActionBuilder trajectory = drive.actionBuilder(startPose)
                .waitSeconds(1)
                .afterTime(0.35, new SequentialAction(
                        chain.scorePosition(),
                        new SleepAction(2),
                        chain.scoreSpecimen(),
                        new SleepAction(0.5)

                ))
                .strafeTo(new Vector2d(8.75, -37))

                .waitSeconds(3)



                .setReversed(true)



//increased all of these by +3
                .splineToConstantHeading(new Vector2d(30, -44), Math.toRadians(0))


                .splineToConstantHeading(new Vector2d(38, -34), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(42, -22), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(49, -7), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(49, -54), Math.toRadians(270))
//
//                .splineToConstantHeading(new Vector2d(44, -20), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(55, -7), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(55, -54), Math.toRadians(270))
//
//                .splineToConstantHeading(new Vector2d(55, -34), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(56, -22), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(63, -7), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(63, -54), Math.toRadians(270))
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
