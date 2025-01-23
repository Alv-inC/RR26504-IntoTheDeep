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
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.MecanumDrive;
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


@TeleOp(name = "pray")
public class ActionTeleoppers extends ActionOpMode {
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

    cameraProcessor processor;




    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);
        //initialize motors
        //init cameras
        processor = new cameraProcessor();

//        new VisionPortal.Builder()
//                .addProcessor(processor)
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .build();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        externTele = new InitializeTeleOp();
       externTele.initialize(hardwareMap);
       externTele.lext.setPosition(0.05);
       externTele.rext.setPosition(0.05);
        externTele.claw.setPosition(0.75);
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

        // Initialize externTele at the class level
          // Initialize using hardwareMap and telemetry

        // Set up vision processor (if necessary)
        // final OpenCVTEST.CameraStreamProcessor processor = new OpenCVTEST.CameraStreamProcessor();
        // VisionPortal visionPortal = new VisionPortal.Builder()
        //        .addProcessor(processor)
        //        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        //        .build();
    }

    @Override
    public void loop() {
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

        // Trigger actions when gamepad1.x is pressed
        if (gamepad1.x) {
            runningActions.add(new SequentialAction(

            new InstantAction(() -> externTele.rotation.setPosition(0.47)),
                    //turret.setTargetPosition(turretPosition);
                    new InstantAction(() -> externTele.lsecondary.setPosition(0.16)),
                    new InstantAction(() -> externTele.rsecondary.setPosition(0.16)),
                    new InstantAction(() -> externTele.primary.setPosition(0.3)),
                    new InstantAction(() -> turret.setTargetPosition(0)),
                    new InstantAction(() -> lift.setTargetPosition(30))

                    //       externTele.claw.setPosition(0.75);
                    //        turret.setTargetPosition(0);
                    //       lift.setTargetPosition(30);
            ));
        }


        //retract to base position
        if(gamepad1.y){
            runningActions.add(new SequentialAction(

                    new InstantAction(() -> externTele.lsecondary.setPosition(0.16)),
                    new InstantAction(() -> externTele.rsecondary.setPosition(0.16)),
                    new InstantAction(() -> externTele.primary.setPosition(0.3)),
                      new InstantAction(() -> externTele.rotation.setPosition(0.47)),
                    new InstantAction(() -> lift.setTargetPosition(744)),
                    new SleepAction(1),
                    new InstantAction(() -> externTele.claw.setPosition(0.75))


            ));
        }
        //transfer



        // Declare a variable to track the current lift position state
         // Start at 0

// In your loop or function where the dpad_up action is checked
        if (gamepad1.dpad_up && !toggle) {  // Check if the button is pressed and toggle is false
            toggle = true;  // Prevents spamming, now the toggle is true
            // Change the lift position state based on the current state
            if (liftPositionState == 0) {
                liftPositionState = 1;  // Move to position 1
                lift.setTargetPosition(835);
            } else if (liftPositionState == 2) {
                liftPositionState = 0;  // Move to position 2
                lift.setTargetPosition(1700);
            } else if (liftPositionState == 1) {
                liftPositionState = 0;  // Move back to position 0
                if(lift.getPosition() > 1000){
                    lift.setTargetPosition(30);
                }

            }
        } else if (!gamepad1.dpad_up && toggle) {  // Button is released and toggle is true
            toggle = false;  // Reset toggle to allow future button presses
        }

        if(gamepad1.b){
            runningActions.add(new SequentialAction(
                    //turret.setTargetPosition(turret.getCurrentPosition()+ processor.calculateTurretAdjustment);
                    //new InstantAction(() -> externTele.rotation.setPosition(externTele.rotation.getPosition()+processor.getServoAdjustment())),
                    new InstantAction(() -> externTele.lsecondary.setPosition(0.125)),
                    new InstantAction(() -> externTele.rsecondary.setPosition(0.125))


//                    new InstantAction(() -> externTele.rsecondary.setPosition(0.7)),
//                    //wait
//                    new InstantAction(() -> externTele.primary.setPosition(0.72)),
//                    new InstantAction(() -> externTele.rotation.setPosition(0.67))
                    //slide raise
                    //driver move bot forward to attach
            ));
        }

        ////score specimen

            if (gamepad1.dpad_down && !clawRotationToggle) {  // Check if the button is pressed and toggle is false
                clawRotationToggle = true;  // Prevent spamming, now the toggle is true
                // Change the claw rotation state based on the current state
                if (clawRotationState == 0) {
                    clawRotationState = 1;
                    externTele.rotation.setPosition(0.18); // Set rotation to 90 degrees (adjust value as needed)
                } else if (clawRotationState == 1) {
                    clawRotationState = 0;
                    externTele.rotation.setPosition(0.47); // Set rotation to 0 degrees (adjust value as needed)
                }
            } else if (!gamepad1.dpad_down && clawRotationToggle) {  // Button is released and toggle is true
                clawRotationToggle = false;  // Reset toggle to allow future button presses
            }

        if (gamepad1.dpad_right && !dPadtoggle) {  // Check if the button is pressed and toggle is false
            dPadtoggle = true;  // Prevents spamming, now the toggle is true
            // Change the lift position state based on the current state
            if (exPositionState == 0){
                exPositionState = 1;
                externTele.lext.setPosition(0.05); // Move to position when toggle is on
                externTele.rext.setPosition(0.05); // Move to position when toggle is on
            } else if (exPositionState == 1) {
                exPositionState = 0;
                externTele.lext.setPosition(0.25); // Move to position when toggle is off
                externTele.rext.setPosition(0.25); // Move to position when toggle is off
                if(lift.getPosition() > 500 && !actionsRunning) {
                    actionsRunning = true;

                    runningActions.add(new SequentialAction(
                            new SleepAction(1.5),
                            new InstantAction(() -> externTele.claw.setPosition(0.75)),
                            new InstantAction(() -> externTele.rotation.setPosition(0.47)),
                            new SleepAction(1),
                            //turret.setTargetPosition(turretPosition);
                            new InstantAction(() -> externTele.lsecondary.setPosition(0.16)),
                            new InstantAction(() -> externTele.rsecondary.setPosition(0.16)),
                            new InstantAction(() -> externTele.primary.setPosition(0.3)),
                            new InstantAction(() -> externTele.lext.setPosition(0.05)),
                            new InstantAction(() -> externTele.rext.setPosition(0.05))
                    ));
                    exPositionState = 1;

                }
            }
        } else if (!gamepad1.dpad_right && dPadtoggle) {  // Button is released and toggle is true
            dPadtoggle = false;  // Reset toggle to allow future button presses
        }
        if (gamepad1.dpad_left && !IDKtoggle) {  // Check if the button is pressed and toggle is false
            IDKtoggle = true;  // Prevents spamming, now the toggle is true
            // Change the lift position state based on the current state
            if (IDKPositionState == 0){
                IDKPositionState = 1;
               lift.setTargetPosition(1700);
            } else if (IDKPositionState == 1) {
                IDKPositionState = 0;
                lift.setTargetPosition(30);

            }
        } else if (!gamepad1.dpad_left && IDKtoggle) {  // Button is released and toggle is true
            IDKtoggle = false;  // Reset toggle to allow future button presses
        }


        if(gamepad1.left_bumper){
            externTele.claw.setPosition(0.75);  // Open the claw
        }


if(gamepad1.left_trigger == 1){
    runningActions.add(new SequentialAction(
            new InstantAction(() ->     turret.setTargetPosition(turret.getCurrentPosition()+processor.getTurretAdjustment())),
            new SleepAction(0.25),
            new InstantAction(() -> turret.setTargetPosition(turret.getCurrentPosition()+processor.getTurretAdjustment())),
            new SleepAction(0.5),
            new InstantAction(() ->     externTele.lext.setPosition(externTele.lext.getPosition()+processor.getExtensionAdjustment())),
            new InstantAction(() ->     externTele.rext.setPosition(externTele.lext.getPosition()+processor.getExtensionAdjustment())),
            new SleepAction(0.5),
            new InstantAction(() -> externTele.rotation.setPosition(externTele.rotation.getPosition()+processor.getServoAdjustment())),
            new SleepAction(0.5),
            new InstantAction(() -> externTele.lsecondary.setPosition(0.15)),
            new InstantAction(() -> externTele.rsecondary.setPosition(0.15)),
            new SleepAction(0.5),
            new InstantAction(() ->     externTele.claw.setPosition(0.66))
    ));
}


        if(gamepad1.right_bumper){
            externTele.claw.setPosition(0.65);  // Close the claw
            if(externTele.lsecondary.getPosition() > 0.15 && externTele.rsecondary.getPosition() > 0.15){
                runningActions.add(new SequentialAction(
                new InstantAction(() -> lift.setTargetPosition(1150)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.17)),
                        new InstantAction(() -> externTele.rsecondary.setPosition(0.17)),
                        //wait
                        new InstantAction(() -> externTele.primary.setPosition(0.2)),
                        new InstantAction(() -> externTele.rotation.setPosition(0.47)),
                        new InstantAction(() -> externTele.lext.setPosition(0.12)),
                        new InstantAction(() -> externTele.rext.setPosition(0.12))
                        ));
            }
        }
//        if (gamepad1.left_bumper && !clawtoggle) {  // Check if the button is pressed and toggle is false
//            clawtoggle = true;  // Prevents spamming, now the toggle is true
//            // Change the lift position state based on the current state
//            if (clawPositionState == 0) {
//                clawPositionState = 1;
//                externTele.claw.setPosition(0.18);  // Close the claw
//            } else if (clawPositionState == 1) {
//                clawPositionState = 0;
//                externTele.claw.setPosition(0.6);  // Open the claw
//            }
//        } else if (!gamepad1.left_bumper && clawtoggle) {  // Button is released and toggle is true
//            clawtoggle = false;  // Reset toggle to allow future button presses
//        }



        //make trigger change the rotation as you hold, left trigger rotate left, right trigger rotate right



        //
        if(gamepad1.a){
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> externTele.lsecondary.setPosition(0.22)),
                    new InstantAction(() -> externTele.rsecondary.setPosition(0.22)),
                    new InstantAction(() -> externTele.primary.setPosition(0.67))


                        ));
                    }
        if(gamepad2.x){
            RANGE_HIGH = new Scalar(100, 150, 255, 255);
            RANGE_LOW = new Scalar(10, 30, 140, 0);
        }
        if(gamepad2.b){
            RANGE_HIGH = new Scalar(255, 5, 230,255);
            RANGE_LOW = new Scalar(150, 0, 0,0);
        }


        //drivetrain code
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        drive.updatePoseEstimate();


        turret.update();
        lift.update();
    }
}