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
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    boolean toggle = false;
    // Declare toggle variables in your initialization method
    // Declare the toggle states and previous states for buttons
    boolean toggleLift = false;
    boolean toggleArm = false;
    boolean toggleClaw = false;

//    boolean previousDpadUpState = false; // For tracking previous state of the dpad_up button
//    boolean previousDpadLeftState = false; // For tracking previous state of the dpad_left button
//    boolean previousDpadRightState = false; // For tracking previous state of the dpad_right button
//    boolean previousLeftBumperState = false; // For tracking previous state of left bumper

    // Declare the states of each mechanism (lift, arm, claw)
    boolean clawOpen = false;  // State of the claw, true for open, false for closed
    boolean previousBButtonState = false;
    int liftPositionState = 0; // Lift states: 0 = lowest, 1 = mid, 2 = highest
    int armPositionState = 0;  // Arm states: 0 = retracted, 1 = extended
    // Declare the toggle state and previous state for dpad_right
    boolean toggleState = false;  // Toggle state for the specimen mechanism
    public static boolean dPadtoggle = false;
    public static int exPositionState;
    public static boolean clawtoggle;
    public static int clawPositionState;
    private boolean actionsRunning;
    private boolean IDKtoggle;
    private int IDKPositionState;
    private boolean clawRotationToggle;
    private int clawRotationState;
    private MecanumDrive drive;
    boolean joystick = true;
    cameraProcessor processor;




    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);
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
        lift.setTargetPosition(30);

        // Re-initialize the variables to reset their values
        liftPositionState = 0; // Lift states: 0 = lowest, 1 = mid, 2 = highest
        armPositionState = 0;  // Arm states: 0 = retracted, 1 = extended

        toggleState = false;  // Toggle state for the specimen mechanism
        dPadtoggle = false;   // Previous state for dpad_right
        exPositionState = 0;
        clawtoggle = false;
        clawPositionState = 0;
        IDKtoggle = false;
        IDKPositionState = 0;
        clawRotationToggle = false;
        clawRotationState = 0;

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

        //drivetrain code
        if(joystick) {
            if (gamepad1.left_trigger > 0) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ), 3);
            } else {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ), 1.5);
            }
        }
        else {
            if (gamepad1.dpad_down) runningActions.add(chain.rotateBot(drive, -45));
            else if (gamepad1.dpad_up) runningActions.add(chain.rotateBot(drive, 45));
            else if (gamepad1.dpad_right) runningActions.add(chain.rotateBot(drive, -90));
            else if (gamepad1.dpad_left) runningActions.add(chain.rotateBot(drive, 90));
            //else if (gamepad2.dpad_left) runningActions.add(chain.rotate2(drive, 45));
            else if(gamepad2.dpad_down) runningActions.add(chain.strafeHorizontal(drive, -60));
            else if(gamepad2.dpad_up) runningActions.add(chain.strafeHorizontal(drive, 60));
            else if(gamepad2.dpad_right) runningActions.add(chain.strafeVertical(drive, -25));
            else if(gamepad2.dpad_left) runningActions.add(chain.strafeVertical(drive, 25));

        }
        drive.updatePoseEstimate();

        if (gamepad1.left_stick_button) joystick = true;
        if(gamepad1.right_stick_button) joystick = false;

        if(gamepad1.left_bumper){
            externTele.claw.setPosition(0.5);  // Open the claw
        }

        if(gamepad1.right_bumper){
            externTele.claw.setPosition(0.4);  // Close the claw
//            if(externTele.lsecondary.getPosition() > 0.16 && externTele.rsecondary.getPosition() > 0.16){
//                runningActions.add(new SequentialAction(
//                        chain.scorePosition()
//                ));
//            }
        }

//        //hang go forward
//        if(gamepad1.right_trigger > 0){
//            externTele.hang.setPower(gamepad1.right_trigger);
//        }
//
//        //hang go backward
//        if(gamepad1.left_trigger > 0){
//            externTele.hang.setPower(-gamepad1.left_trigger);
//        }
        if(gamepad2.right_trigger>0)runningActions.add(chain.startPosition(false));

        if(gamepad2.left_bumper){
            runningActions.add(new SequentialAction(
                       chain.scorePositionSample()
                ));
        }

        if(gamepad2.right_stick_button){
            runningActions.add(new SequentialAction(
                    chain.intakePosition()
            ));
        }

        boolean previousButtonState2a = false; // Stores the previous state of the button
        if (gamepad2.a && !previousButtonState2a) {
            runningActions.add(new SequentialAction(
                chain.intake(processor)
            ));
        }
        previousButtonState2a = gamepad2.a;

        if(gamepad2.right_bumper) externTele.claw.setPosition(0.5);  // Open the claw

        else if(gamepad2.left_stick_button){
            runningActions.add(new SequentialAction(
                    chain.startPosition(true)
            ));
        }

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



        turret.update();
        lift.update();
        telemetry.addData("UPDATED", 0);
        telemetry.addData("rotation adjustment", processor.getServoAdjustment());
        telemetry.addData("rotation position", externTele.rotation.getPosition());
        telemetry.addData("turret adjustment", processor.getTurretAdjustment());
        telemetry.addData("extension adjustment", processor.getExtensionAdjustment());
    }


}