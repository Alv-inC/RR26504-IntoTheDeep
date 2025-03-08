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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ChainActions;
import org.firstinspires.ftc.teamcode.Subsystems.IDK;
import org.firstinspires.ftc.teamcode.Subsystems.InitializeTeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.cameraProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.Arrays;

@Autonomous(name = "BASKETTTTT")
public class basketAuto extends LinearOpMode {
    private Lift lift;
    private Turret turret;
    // private ChainActions chain = new ChainActions(hardwareMap);
    private InitializeTeleOp externTele;
    private ChainActions chain;
    private cameraProcessor processor;
    private double rot, tur, ext = 0;

    VelConstraint slower = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(30),
            new AngularVelConstraint(Math.PI / 2)
    ));

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-38, -62.75, Math.toRadians(90)));
        Pose2d startPose = new Pose2d(-38, -62.75, Math.toRadians(90));
        externTele = new InitializeTeleOp();
        lift = new Lift(hardwareMap, 1);
        turret = new Turret(hardwareMap, 1);
        chain = new ChainActions(hardwareMap);
        externTele.initialize(hardwareMap);
        externTele.lext.setPosition(0);
        externTele.rext.setPosition(0);
        externTele.claw.setPosition(0.42);
        turret.setTargetPosition(0);
        lift.setTargetPosition(30);

        processor = new cameraProcessor(new Scalar(100, 0, 0,0), new Scalar(255, 5, 230,255), true);
//        VisionPortal visionPortal = new VisionPortal.Builder()
//                .addProcessor(processor)
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .build();
//
//// Start a background thread to update cameraProcessor values
//        Thread visionThread = new Thread(() -> {
//            while (!isStopRequested()) {  // Keep running until OpMode stops
//                rot = processor.getServoAdjustment();
//                ext = processor.getExtensionAdjustment();
//                tur = processor.getTurretAdjustment();
//                FtcDashboard.getInstance().sendImage(processor.getMaskedFrameBitmap());
//                telemetry.addData("Detected Value", rot);  // Debugging
//                telemetry.update();
//            }
//        });
//        visionThread.start();

        waitForStart();

        if (isStopRequested()) return;

        TrajectoryActionBuilder trajectory = drive.actionBuilder(startPose)


//get ready to score & after time, score
// robot rotates to get new sample after 'scoring', may need to rotate turret to correct for incorrect angle

                .afterTime(0, new SequentialAction(
                        new InstantAction(() -> turret.setTargetPosition(-1250)),
                  chain.lowBasketPosition(),
                  new SleepAction(1.5),
                  chain.scoreBasket(),
                        new InstantAction(() -> turret.setTargetPosition(0)),
                  new SleepAction(1.5),
                  chain.intakePosition()
                ))
                .splineToLinearHeading(new Pose2d(-54.5, -52, Math.toRadians(65)), Math.toRadians(140))
                .waitSeconds(6) //score after

                //SCORE
                .splineToLinearHeading(new Pose2d(-55, -40, Math.toRadians(65)), Math.toRadians(90))
                .waitSeconds(1.5)
                .afterTime(0, new SequentialAction(
                        new InstantAction(() -> lift.setTargetPosition(0)),
                        new SleepAction(0.5),
                        new InstantAction(() ->externTele.claw.setPosition(0.55))
                ))
                .waitSeconds(3)
                .splineToLinearHeading(new Pose2d(-54.5, -52, Math.toRadians(65)), Math.toRadians(270))
                //score
                .afterTime(0, new SequentialAction(

                        new InstantAction(() -> externTele.lext.setPosition(0)),
                        new InstantAction(() -> externTele.rext.setPosition(0)),
                        new InstantAction(()-> externTele.rotation.setPosition(0.48)),
                        new InstantAction(() -> externTele.lsecondary.setPosition(0.34)),
                        new InstantAction(() -> externTele.rsecondary.setPosition(0.34)),
                        new InstantAction(() -> externTele.primary.setPosition(0.71)),
                        new SleepAction(0.5),
                        new InstantAction(() -> turret.setTargetPosition(-1250)),
                        chain.lowBasketPosition(),
                        new SleepAction(2),
                        chain.scoreBasket(),
                        new InstantAction(() -> externTele.lext.setPosition(0)),
                        new InstantAction(() -> externTele.rext.setPosition(0)),
                        new InstantAction(()-> externTele.rotation.setPosition(0.48)),
                        new InstantAction(() -> externTele.lsecondary.setPosition(0.34)),
                        new InstantAction(() -> externTele.rsecondary.setPosition(0.34)),
                        new InstantAction(() -> externTele.primary.setPosition(0.71)),
                        new InstantAction(() -> turret.setTargetPosition(0)),
                        chain.intakePosition()
                ))

                //grab
                .waitSeconds(5)
                //new score position
                .splineToLinearHeading(new Pose2d(-60, -40, Math.toRadians(90)), Math.toRadians(140))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-54.5, -52, Math.toRadians(65)), Math.toRadians(140))
                //SCORE

                //grab
                .waitSeconds(3)
                //new score position
                .splineToLinearHeading(new Pose2d(-60, -40, Math.toRadians(110)), Math.toRadians(140))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-54.5, -52, Math.toRadians(65)), Math.toRadians(140))
                .waitSeconds(3)
                //SCORE

                //grab

                //grab from summersible
                .splineToLinearHeading(new Pose2d(-25, -12, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(3)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-54.5, -52, Math.toRadians(65)), Math.toRadians(180))






                .endTrajectory();

        Action trajectoryAction = trajectory.build();

        Actions.runBlocking(new ParallelAction(
                trajectoryAction,
                lift.updateAction(),
                turret.updateAction()
        ));
    }
    }