package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.PI;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ChainActions;
import org.firstinspires.ftc.teamcode.Subsystems.InitializeTeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import page.j5155.expressway.ftc.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.cameraProcessor;


@TeleOp(name = "Sample Pray")
public class BasketTeleOp extends ActionOpMode {
    public static boolean clawgo, rotationgo, extensiongo, secondarygo = false;
    public static double width, height = 0;
    public static boolean MASK_TOGGLE = true;
    //    public static Scalar RANGE_LOW = new Scalar(170, 100, 0, 0);
//    public static Scalar RANGE_HIGH = new Scalar(255, 255, 230, 255);
    public static Scalar RANGE_LOW = new Scalar(0, 0, 0, 0);   // Minimum HSV values
    public static Scalar RANGE_HIGH = new Scalar(180, 255, 255, 255); // Maximum HSV values

    public static double rotpos = 0.5;


    private InitializeTeleOp externTele;  // Use the class-level reference

    private Lift lift;
    private Turret turret;
    private ChainActions chain;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    int liftPositionState = 0; // Lift states: 0 = lowest, 1 = mid, 2 = highest
    int armPositionState = 0;  // Arm states: 0 = retracted, 1 = extended
    // Declare the toggle state and previous state for dpad_right
    boolean toggleState = false;  // Toggle state for the specimen mechanism
    public static boolean dPadtoggle = false;
    public static int exPositionState;
    public static boolean clawtoggle;
    public static int clawPositionState;
    private MecanumDrive drive;

    cameraProcessor processor;
    boolean flag = false;
    boolean previousButtonState2a = false;



    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lift = new Lift(hardwareMap, 1);
        turret = new Turret(hardwareMap, 1);
        chain = new ChainActions(hardwareMap);
        //initialize motors
        //init cameras

        processor = new cameraProcessor(new Scalar(100, 0, 0,0), new Scalar(255, 5, 230,255), false);

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        externTele = new InitializeTeleOp();
        externTele.initialize(hardwareMap);
        turret.setTargetPosition(0);
        lift.setTargetPosition(0);
        externTele.claw.setPosition(0.8);

        // Re-initialize the variables to reset their values
        liftPositionState = 0; // Lift states: 0 = lowest, 1 = mid, 2 = highest
        armPositionState = 0;  // Arm states: 0 = retracted, 1 = extended

        toggleState = false;  // Toggle state for the specimen mechanism
        dPadtoggle = false;   // Previous state for dpad_right
        exPositionState = 0;
        clawtoggle = false;
        clawPositionState = 0;
    }

    @Override
    public void loop() {

        if(MASK_TOGGLE){
            FtcDashboard.getInstance().sendImage(processor.getMaskedFrameBitmap());
        } else {
            FtcDashboard.getInstance().sendImage(processor.getLastFrame());
        }
        // Update running actions
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        ///PLAYER 1 CODE
        //drivetrain code
        if(turret.getCurrentPosition()>=0 && turret.getCurrentPosition()<=50){
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ), gamepad2.left_trigger>0 || gamepad1.left_trigger>0 || lift.getPosition()>50 ? 2.3 : 1.3);
        }
        else {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x
                    ),
                    gamepad1.right_stick_x
            ), gamepad2.left_trigger > 0 || gamepad1.left_trigger > 0 || lift.getPosition() > 50 ? 2.3 : 1.3);
        }

        drive.updatePoseEstimate();

        // Trigger actions when gamepad1.x is pressed
        if (gamepad1.x) {
            flag = true;
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> externTele.lsecondary.setPosition(0.34)),
                    new InstantAction(() -> externTele.rsecondary.setPosition(0.34)),
                    new InstantAction(() -> externTele.primary.setPosition(0.7)),
                    new InstantAction(() -> externTele.lext.setPosition(0)),
                    new InstantAction(() -> externTele.rext.setPosition(0)),
                    new InstantAction(() -> externTele.rotation.setPosition(0.48)),
                    new InstantAction(() -> lift.setTargetPosition(0)),
                    new SleepAction(0.5),
                    new InstantAction(() -> turret.setTargetPosition(-1250))
            ));
        }

        if(gamepad1.y){
            runningActions.add(new SequentialAction(
                    chain.scoreLowBasket()
            ));
        }

        if(gamepad1.left_bumper) externTele.claw.setPosition(0.8);  // Open the claw
        if(gamepad1.right_bumper)externTele.claw.setPosition(0.55);  // Close the claw

        if(gamepad1.dpad_up) lift.setTargetPosition(945);

/// PLAYER 2 CODE
        if(gamepad2.dpad_down){
            runningActions.add(new SequentialAction(
                    chain.intakePosition()
            ));
        }
        if(gamepad2.dpad_up)lift.setTargetPosition(800);

        if(gamepad2.right_stick_button){
            runningActions.add(new SequentialAction(
                    chain.startPosition(false)
            ));
        }
        if(gamepad2.right_bumper){
            externTele.lext.setPosition(0.15);
            externTele.rext.setPosition(0.15);
        }
        if (gamepad2.a && !previousButtonState2a) {
            runningActions.add(new SequentialAction(
                    chain.intake(processor)
            ));
        }
        previousButtonState2a = gamepad2.a;


        if(gamepad2.x){
            processor.RANGE_HIGH_1 = new Scalar(100, 150, 255, 255);
            processor.RANGE_LOW_1 = new Scalar(10, 30, 140, 0);
        }
        if(gamepad2.b){
            processor.RANGE_HIGH_1 = new Scalar(255, 5, 230,255);
            processor.RANGE_LOW_1 = new Scalar(150, 0, 0,0);
        }
        if(gamepad2.y){
            processor.yellow = true;
        }

        boolean buttonState2a = false;
        buttonState2a = gamepad2.left_stick_button;

        turret.update();
        lift.update();
        telemetry.addData("UPDATED", 0);
        telemetry.addData("rotation adjustment", processor.getServoAdjustment());
        telemetry.addData("rotation position", externTele.rotation.getPosition());
        telemetry.addData("turret adjustment", processor.getTurretAdjustment());
        telemetry.addData("extension adjustment", processor.getExtensionAdjustment());
    }



}