package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.cameraProcessor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ChainActions {
    //initialize stuffs
    private Lift lift;
    private Turret turret;
    private InitializeTeleOp externTele;
    TrajectoryActionBuilder turnBot, strafeH, strafeV, basket, intake;
    public ChainActions(HardwareMap hardwareMap){
        lift = new Lift(hardwareMap, 1);
        turret = new Turret(hardwareMap, 1);
        externTele = new InitializeTeleOp();
        externTele.initialize(hardwareMap);
    }

    public Action rotate2(MecanumDrive drive, int angle){
        turnBot = drive.actionBuilder(new Pose2d(0, 0, drive.pose.heading.toDouble()))
                .turnTo(Math.toRadians(angle))
                .endTrajectory();
        return turnBot.build();
    }

    public Action rotateBot(MecanumDrive drive, int rotateTicks){
        turnBot = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                .turn(Math.toRadians(rotateTicks))
                .endTrajectory();
        return turnBot.build();
    }
    public Action strafeHorizontal(MecanumDrive drive, int h){
        strafeH = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                .strafeTo(new Vector2d(drive.pose.position.x+h, drive.pose.position.y))
                .endTrajectory();
        return strafeH.build();
    }
    public Action strafeVertical(MecanumDrive drive, int v){
        strafeV = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y+v))
                .endTrajectory();
        return strafeV.build();
    }
    public Action basketPosition(MecanumDrive drive){
        basket = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble()))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(drive.pose.position.x - 34, drive.pose.position.y - 37, Math.toRadians(45)), Math.toRadians(180))
                .endTrajectory();
        return basket.build();
    }



    public Action startPosition(boolean pause){
        return pause? new SequentialAction(
                //add the chain of commands here
                new InstantAction(() -> externTele.claw.setPosition(0.95)),
                new InstantAction(() -> externTele.lext.setPosition(0.05)),
                new InstantAction(() -> externTele.rext.setPosition(0.05)),
                new InstantAction(() -> externTele.rotation.setPosition(0.47)),
                new InstantAction(() -> externTele.primary.setPosition(0.33)),
                new SleepAction(0.5) ,
                new InstantAction(() -> externTele.lsecondary.setPosition(0.34)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.34)),
                new InstantAction(() -> lift.setTargetPosition(0)),
                new SleepAction(1.5),
                new InstantAction(() -> turret.setTargetPosition(0))
        )
                :
                new SequentialAction(
                        new InstantAction(() -> externTele.lext.setPosition(0)),
                        new InstantAction(() -> externTele.rext.setPosition(0)),
                        new InstantAction(()-> externTele.rotation.setPosition(0.48)),
                        new InstantAction(() -> externTele.lsecondary.setPosition(0.34)),
                        new InstantAction(() -> externTele.rsecondary.setPosition(0.34)),
                        new InstantAction(() -> externTele.primary.setPosition(0.71)),
                        new InstantAction(() -> turret.setTargetPosition(0)),
                        new InstantAction(() -> lift.setTargetPosition(0))

                );
    }

    //
    public Action scorePosition(){
        return new SequentialAction(
                new InstantAction(() -> externTele.primary.setPosition(0.4)),
                new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                new InstantAction(() -> externTele.lext.setPosition(0)),
                new InstantAction(() -> externTele.rext.setPosition(0)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.37)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.37)),
                new SleepAction(0.3),
                new InstantAction(() -> turret.setTargetPosition(-1250)),
                new SleepAction(1.6),
                new InstantAction(() -> externTele.primary.setPosition(0.71)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.38)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.38)),
                new InstantAction(() -> lift.setTargetPosition(1035))
        );
    }
    public Action scorePositionTeleop(){
        return new SequentialAction(
                new InstantAction(() -> externTele.lsecondary.setPosition(0.3)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.3)),
                new InstantAction(() -> externTele.primary.setPosition(0.25)),
                new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                new SleepAction(0.5),
                new InstantAction(() -> turret.setTargetPosition(0)),
                new SleepAction(1),
                new InstantAction(() -> externTele.primary.setPosition(0.68)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.38)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.38))
        );
    }
    public Action scorePositionSample(){
        return new SequentialAction(
                new InstantAction(()->turret.setTargetPosition(-1250)),
                new SleepAction(1.5),
                new InstantAction(()->lift.setTargetPosition(2100)),
                new InstantAction(()->externTele.lsecondary.setPosition(0.37)),
                new InstantAction(()->externTele.rsecondary.setPosition(0.37)),
                new SleepAction(1.5),
                new InstantAction(()->externTele.primary.setPosition(0.43))
        );
    }

    public Action scoreSpecimen(){
        return new SequentialAction(
                new InstantAction(() -> lift.setTargetPosition(280)),
                new SleepAction(0.71),
                new InstantAction(() -> externTele.claw.setPosition(0.8))
        );
    }

    public Action ScoreAndDown(){
        return new SequentialAction(
                new InstantAction(() -> externTele.claw.setPosition(0.73))

        );
    }

    public Action grabPosition(){
        return new SequentialAction(
                new InstantAction(() -> externTele.lext.setPosition(0)),
                new InstantAction(() -> externTele.rext.setPosition(0)),
                new InstantAction(() -> externTele.primary.setPosition(0.34)),
                new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.41)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.41)),
                new SleepAction(0.3),
                new InstantAction(() -> turret.setTargetPosition(0)),
                new SleepAction(1.7),
                new InstantAction(() -> lift.setTargetPosition(0)),
                new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.155)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.155)),
                new InstantAction(() -> externTele.primary.setPosition(0.71)),
                new InstantAction(() -> externTele.claw.setPosition(0.95))


        );
    }


    public Action grabPositionBound() {
        return new SequentialAction(
                new InstantAction(() -> lift.setTargetPosition(0)),
                new InstantAction(() -> externTele.lext.setPosition(0)),
                new InstantAction(() -> externTele.rext.setPosition(0)),
                new InstantAction(() -> externTele.primary.setPosition(0.34)),
                new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.41)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.41)),
                new SleepAction(1),
                new InstantAction(() -> turret.setTargetPosition(-1250)),
                new SleepAction(1.7),
                new InstantAction(() -> lift.setTargetPosition(0)),
                new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.155)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.155)),
                new InstantAction(() -> externTele.primary.setPosition(0.71)),
                new InstantAction(() -> externTele.claw.setPosition(0.95))


        );
    }
    public Action intakeToGrab(){
        lift.divide = 1.3;
        return new SequentialAction(
                new InstantAction(() -> externTele.lext.setPosition(0)),
                new InstantAction(() -> externTele.rext.setPosition(0)),
                new InstantAction(() -> lift.setTargetPosition(0)),
                new SleepAction(0.2),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.3)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.3)),
                new InstantAction(() -> externTele.primary.setPosition(0.85)),
                new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                new SleepAction(2),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.155)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.155)),
                new InstantAction(() -> externTele.primary.setPosition(0.7))
        );
    }
    public Action intakeToBasket(){
        lift.divide = 1.3;
        return new SequentialAction(
                new InstantAction(() -> externTele.lext.setPosition(0)),
                new InstantAction(() -> externTele.rext.setPosition(0)),
                new InstantAction(() -> lift.setTargetPosition(0)),
                new SleepAction(0.2),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.32)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.32)),
                new InstantAction(() -> externTele.primary.setPosition(0.85)),
                new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                new SleepAction(2),
                new InstantAction(() -> externTele.primary.setPosition(0.3))
        );
    }
    public Action intakePosition(){
        lift.divide = 1.7;
        return new SequentialAction(
                new InstantAction(() -> turret.setTargetPosition(0)),
                new InstantAction(() -> lift.setTargetPosition(600)),
                new InstantAction(() -> externTele.claw.setPosition(0.95)),
                new InstantAction(() -> externTele.lext.setPosition(0.04)),
                new InstantAction(() -> externTele.rext.setPosition(0.04)),
                new InstantAction(() -> externTele.primary.setPosition(1)),
                new InstantAction(() -> externTele.rotation.setPosition(0.47)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.09)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.09))
        );
    }

    public Action grabPositionAuto(){
        return new SequentialAction(
                new InstantAction(() -> externTele.lext.setPosition(0.05)),
                new InstantAction(() -> externTele.rext.setPosition(0.05)),
                new InstantAction(() -> externTele.primary.setPosition(0.34)),
                new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.41)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.41)),
                new SleepAction(0.3),
                new InstantAction(() -> turret.setTargetPosition(-1250)),
                new SleepAction(1.7),
                new InstantAction(() -> lift.setTargetPosition(0)),
                new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.155)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.155)),
                new InstantAction(() -> externTele.primary.setPosition(0.6))

        );
    }

    public Action scorePositionAuto(){
        return new SequentialAction(
                new InstantAction(() -> externTele.primary.setPosition(0.4)),
                new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                new InstantAction(() -> externTele.lext.setPosition(0)),
                new InstantAction(() -> externTele.rext.setPosition(0)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.37)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.37)),
                new SleepAction(0.3),
                new InstantAction(() -> turret.setTargetPosition(0)),
                new SleepAction(1.6),
                new InstantAction(() -> externTele.primary.setPosition(0.68)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.38)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.38)),
                new InstantAction(() -> lift.setTargetPosition(945))
        );
    }
    public Action grabSample(cameraProcessor processor){
        return new SequentialAction(
                new InstantAction(() ->externTele.rotation.setPosition(externTele.rotation.getPosition()+processor.getServoAdjustment())),
                new SleepAction(0.3),
                new InstantAction(() ->externTele.lsecondary.setPosition(0.16)),
                new InstantAction(() ->externTele.rsecondary.setPosition(0.16)),
                new SleepAction(0.4),
                new InstantAction(() ->externTele.claw.setPosition(0.42))
        );
    }

    public Action readyGrab(){
        return new SequentialAction(
                new InstantAction(() -> externTele.claw.setPosition(0.6)),
                new InstantAction(() -> externTele.lext.setPosition(0.33)),
                new InstantAction(() -> externTele.rext.setPosition(0.33)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.28)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.28)),
                new InstantAction(() -> externTele.primary.setPosition(0.97)),
                new InstantAction(() -> externTele.rotation.setPosition(0.47))
        );
    }

    public Action readyGrabAuto(){
        return new SequentialAction(
                new InstantAction(() -> externTele.claw.setPosition(0.6)),
                new SleepAction(1.5),
                new InstantAction(() -> lift.setTargetPosition(0)),
                new InstantAction(() -> externTele.lext.setPosition(0.33)),
                new InstantAction(() -> externTele.rext.setPosition(0.33)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.28)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.28)),
                new InstantAction(() -> externTele.primary.setPosition(0.97)),
                new InstantAction(() -> externTele.rotation.setPosition(0.47))
        );
    }
    public Action scoreBasket(){
        return new SequentialAction(
                new InstantAction(() -> externTele.claw.setPosition(0.9)),
                new SleepAction(0.3),
                new InstantAction(() -> externTele.primary.setPosition(0.3)),
                new InstantAction(() -> externTele.lext.setPosition(0)),
                new InstantAction(() -> externTele.rext.setPosition(0)),
                new SleepAction(0.7),
                new InstantAction(() -> lift.setTargetPosition(0))
        );
    }
    public Action highBasketPosition(){
        lift.divide = 1.5;
        return new SequentialAction(
                new InstantAction(() -> lift.setTargetPosition(2250)),
                new SleepAction(1),
                new InstantAction(() -> externTele.primary.setPosition(0.5)),
                new SleepAction(0.3),
                new InstantAction(() -> externTele.lext.setPosition(0.15)),
                new InstantAction(() -> externTele.rext.setPosition(0.15))
        );
    }
    public Action lowBasketPosition(){
        lift.divide = 1.5;
        return new SequentialAction(
                new InstantAction(() -> lift.setTargetPosition(1200)),
                new SleepAction(1),
                new InstantAction(() -> externTele.primary.setPosition(0.65)),
                new SleepAction(0.3),
                new InstantAction(() -> externTele.lext.setPosition(0.1)),
                new InstantAction(() -> externTele.rext.setPosition(0.1))
        );
    }
    public Action intake(cameraProcessor processor, boolean flag){
        intakePosition();
        lift.divide = 2;
        double rotAdjust = processor.getServoAdjustment();
        double turAdjust = processor.getTurretAdjustment();
        double extAdjust = processor.getExtensionAdjustment();

        return flag ? new SequentialAction(
                new InstantAction(() ->externTele.rotation.setPosition(0.48+rotAdjust)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.09)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.09)),
                new SleepAction(1),
                new InstantAction(() ->externTele.claw.setPosition(0.55))
        ) : new SequentialAction(
                new InstantAction(() ->turret.setTargetPosition(turAdjust)),
                new InstantAction(() ->externTele.lext.setPosition(externTele.lext.getPosition()+extAdjust)),
                new InstantAction(() ->externTele.rext.setPosition(externTele.lext.getPosition()+extAdjust)),
                new SleepAction(0.5),
                new InstantAction(() ->externTele.rotation.setPosition(0.47+rotAdjust)),
                new InstantAction(() -> lift.setTargetPosition(0)),
                new SleepAction(0.5),
                new InstantAction(() ->externTele.claw.setPosition(0.55))
        );
    }







    //prepareGrab
    //Grab
    //neutralPosition
//






}