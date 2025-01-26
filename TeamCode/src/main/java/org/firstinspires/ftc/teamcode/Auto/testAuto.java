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
        externTele.lext.setPosition(0.05);
        externTele.rext.setPosition(0.05);
        externTele.claw.setPosition(0.55);
        turret.setTargetPosition(0);
        lift.setTargetPosition(30);



        waitForStart();
        if(isStopRequested()) return;

        TrajectoryActionBuilder trajectory = drive.actionBuilder(startPose)
                .waitSeconds(1)

                //uncomment later; it works but inconsistent
//                .afterTime(0.35, new SequentialAction(
//                        chain.scorePosition(),
//                        new SleepAction(2),
//                        chain.scoreSpecimen(),
//                        new SleepAction(0.5)
//
//                ))
                .strafeTo(new Vector2d(8.75, -37))

                .waitSeconds(3)



                .setReversed(true)



//increased all of these by +3
                .splineToConstantHeading(new Vector2d(31.5, -44), Math.toRadians(0))

                //first specimen
                .splineToConstantHeading(new Vector2d(39.5, -33), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(43.5, -21), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(49.5, -5), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(49.5, -61), Math.toRadians(270))
                .waitSeconds(0.3)

                //second specimen
                .splineToConstantHeading(new Vector2d(50, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60.5, -5), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(60.5, -61), Math.toRadians(270))
                .waitSeconds(0.3)

//
//                //third specimen
//                .splineToConstantHeading(new Vector2d(52.5, -33), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(52.5, -21), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(55, -5), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(58.5, -61), Math.toRadians(270))
//                .waitSeconds(0.3)

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
