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
import org.firstinspires.ftc.teamcode.Subsystems.cameraProcessor;
import org.opencv.core.Scalar;

import java.util.Arrays;

@Autonomous(name = "strafe auto")
public class strafeAuto extends LinearOpMode {
    private Lift lift;
    private Turret turret;
    // private ChainActions chain = new ChainActions(hardwareMap);
    private InitializeTeleOp externTele;
    private ChainActions chain;
    private cameraProcessor processor;

    VelConstraint slower = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(30),
            new AngularVelConstraint(Math.PI / 2)
    ));

    @Override
    public void runOpMode() throws InterruptedException {
        processor = new cameraProcessor(new Scalar(100, 0, 0,0), new Scalar(255, 5, 230,255), true);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-38, -62.75, Math.toRadians(90)));
        Pose2d startPose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(90));
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
                .waitSeconds(3)


//get ready to score & after time, score
// robot rotates to get new sample after 'scoring', may need to rotate turret to correct for incorrect angle

                .afterTime(0, new SequentialAction(chain.strafeVertical(drive, 23)))
                .afterTime(2, new SequentialAction(turret.setTargetPositionAction(0)))
                .afterTime(1, new SequentialAction(chain.strafeVertical(drive, -23)))
                .afterTime(2, new SequentialAction(chain.rotateBot(drive, -45)))
                .afterTime(1, new SequentialAction(chain.rotateBot(drive, 45)))

                .afterTime(0, new SequentialAction(chain.strafeVertical(drive, 23)))
                .afterTime(2, new SequentialAction(turret.setTargetPositionAction(300)))
                .afterTime(1, new SequentialAction(turret.setTargetPositionAction(0)))
                .afterTime(1, new SequentialAction(chain.strafeVertical(drive, -23)))
                .afterTime(2, new SequentialAction(chain.rotateBot(drive, -45)))
                .afterTime(1, new SequentialAction(chain.rotateBot(drive, 45)))

                .afterTime(0, new SequentialAction(chain.strafeVertical(drive, 23)))
                .afterTime(2, new SequentialAction(turret.setTargetPositionAction(-300)))
                .afterTime(1, new SequentialAction(turret.setTargetPositionAction(0)))
                .afterTime(1, new SequentialAction(chain.strafeVertical(drive, -23)))
                .afterTime(2, new SequentialAction(chain.rotateBot(drive, -45)))
                .afterTime(1, new SequentialAction(chain.rotateBot(drive, 45)))

                .endTrajectory();
        Action trajectoryAction = trajectory.build();

        Actions.runBlocking(new ParallelAction(
                trajectoryAction,
                lift.updateAction(),
                turret.updateAction()
        ));
    }
}