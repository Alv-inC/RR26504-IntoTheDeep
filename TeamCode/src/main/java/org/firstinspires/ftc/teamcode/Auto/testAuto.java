package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ChainActions;
import org.firstinspires.ftc.teamcode.Subsystems.IDK;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@Autonomous(name = "Testing Auto")
public class testAuto extends LinearOpMode {
    private Lift lift = new Lift(hardwareMap, telemetry);
    private ChainActions chain = new ChainActions(hardwareMap, telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));


        waitForStart();
        if(isStopRequested()) return;

        TrajectoryActionBuilder trajectory = drive.actionBuilder(startPose)
                .afterTime(0, lift.setTargetPositionAction(500))
                .endTrajectory();


        Actions.runBlocking(new IDK.RaceParallelCommand(
                trajectory.build(),
                lift.updateAction()
        ));

        Action trajectoryAction = trajectory.build();



    }
}
