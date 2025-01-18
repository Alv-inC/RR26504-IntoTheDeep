package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.PI;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

@TeleOp(name = "pray")
public class ActionTeleoppers extends ActionOpMode {
    public static boolean clawgo, rotationgo, extensiongo, secondarygo = false;
    public static double width, height = 0;
    public static boolean MASK_TOGGLE = true;
    public static Scalar RANGE_LOW = new Scalar(170, 100, 0, 0);
    public static Scalar RANGE_HIGH = new Scalar(255, 255, 230, 255);
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

    boolean previousDpadUpState = false; // For tracking previous state of the dpad_up button
    boolean previousDpadLeftState = false; // For tracking previous state of the dpad_left button
    boolean previousDpadRightState = false; // For tracking previous state of the dpad_right button
    boolean previousLeftBumperState = false; // For tracking previous state of left bumper

    // Declare the states of each mechanism (lift, arm, claw)
    boolean clawOpen = false;  // State of the claw, true for open, false for closed
    int liftPositionState = 0; // Lift states: 0 = lowest, 1 = mid, 2 = highest
    int armPositionState = 0;  // Arm states: 0 = retracted, 1 = extended
    // Declare the toggle state and previous state for dpad_right
    boolean toggleState = false;  // Toggle state for the specimen mechanism
    public static boolean dPadtoggle = false;
    public static int exPositionState = 0;
    public static boolean clawtoggle = false;
    public static int clawPositionState = 0;
    private CameraStreamProcessor processor;


    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {

        Mat peopleMask = new Mat();
        private final AtomicReference<Double> servoAdjustment = new AtomicReference<>(0.0);
        private final AtomicReference<Double> extensionAdjustment = new AtomicReference<>(0.0);
        private final AtomicReference<Double> turretAdjustment = new AtomicReference<>(0.0);
        private final AtomicReference<Scalar> centerHSV = new AtomicReference<>(new Scalar(0, 0, 0, 0));
        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
        private final AtomicReference<Bitmap> peopleMaskedFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }

        public double getServoAdjustment() {
            return servoAdjustment.get();
        }

        public double getExtensionAdjustment() {
            return extensionAdjustment.get();
        }

        public double getTurretAdjustment() {
            return turretAdjustment.get();
        }

        public Bitmap getMaskedFrameBitmap() {
            return peopleMaskedFrame.get();
        }

        public Bitmap getLastFrame() {
            return lastFrame.get();
        }

        public Scalar getCenterHSV() {
            return centerHSV.get();
        }

        @Override
        public void init(int width, int height, CameraCalibration cameraCalibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGBA2RGB);
            int centerX = frame.width() / 2;
            int centerY = frame.height() / 2;
            double[] centerPixelHSV = hsvFrame.get(centerY, centerX);
            centerHSV.set(new Scalar(centerPixelHSV));
            Imgproc.circle(frame, new Point(centerX, centerY), 10, new Scalar(0, 255, 0), 2);

            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            Core.inRange(frame, RANGE_LOW, RANGE_HIGH, peopleMask);
            Bitmap b_peopleMask = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(peopleMask, b_peopleMask);
            peopleMaskedFrame.set(b_peopleMask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(peopleMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest contour by area
            double maxArea = 0;
            MatOfPoint largestContour = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }
            if (largestContour != null) {
                RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
                servoAdjustment.set(calculateServoAdjustment(rotatedRect));
                extensionAdjustment.set(calculateExtensionAdjustment(rotatedRect));
                //turretAdjustment.set(calculateTurretAdjustment(rotatedRect));
            }


            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {

        }

        public double calculateServoAdjustment(RotatedRect rotatedRect) {
            // Calculate the angle of the rectangle if a valid contour is found
            double angle;
            if (rotatedRect.size.width > rotatedRect.size.height) {
                // Swap width and height if needed
                rotatedRect = new RotatedRect(rotatedRect.center,
                        new Size(rotatedRect.size.height, rotatedRect.size.width),
                        rotatedRect.angle + 90);
            }
            width = rotatedRect.size.width;
            height = rotatedRect.size.height;

            if (rotatedRect.size.width > rotatedRect.size.height) {
                // Adjust the angle to represent the longer side (height)
                angle = rotatedRect.angle + 90;
            } else {
                // Width is greater than height; the angle already represents the longer side
                angle = rotatedRect.angle;
            }
// Normalize the angle to fall within a preferred range if necessary
            if (angle >= 90) {
                angle -= 180;
            } else if (angle < -90) {
                angle += 180;
            }
            //convert angle to servo ticks
            return angle /= 300;
        }

        public double calculateExtensionAdjustment(RotatedRect rotatedRect) {
            double contourY = rotatedRect.center.y;
            double offsetY = contourY - 480 / 2;
            double offsetMM = offsetY * 152 / 480;
            double offsetTicks = offsetMM * 0.33 / 303;
            offsetTicks *= -1;
            offsetTicks /= 2;
            offsetTicks += 0.03;
            if (offsetTicks < 0) offsetTicks -= 0.01
                    ;
            //else offsetTicks += 0.03;
            return offsetTicks;
        }
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);
        //initialize motors
        //init cameras
        processor = new CameraStreamProcessor();

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        fr = hardwareMap.get(DcMotor.class, "rightFront");
        fl = hardwareMap.get(DcMotor.class, "leftFront");
        br = hardwareMap.get(DcMotor.class, "rightBack");
        bl = hardwareMap.get(DcMotor.class, "leftBack");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        externTele = new InitializeTeleOp();
       externTele.initialize(hardwareMap, telemetry);
       turret.setTargetPosition(0);
       lift.setTargetPosition(0);

        // Re-initialize the variables to reset their values
        clawOpen = false;  // State of the claw, true for open, false for closed
        liftPositionState = 0; // Lift states: 0 = lowest, 1 = mid, 2 = highest
        armPositionState = 0;  // Arm states: 0 = retracted, 1 = extended

        toggleState = false;  // Toggle state for the specimen mechanism
        dPadtoggle = false;   // Previous state for dpad_right
        exPositionState = 0;
        clawtoggle = false;
        clawPositionState = 0;

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
                    new InstantAction(() -> externTele.primary.setPosition(0.53)),
                    new InstantAction(() -> externTele.lext.setPosition(0)),
                    new InstantAction(() -> externTele.rext.setPosition(0))
            ));
        }


        //retract to base position
        if(gamepad1.y){
            runningActions.add(new SequentialAction(

                    new InstantAction(() -> externTele.lsecondary.setPosition(0.22)),
                    new InstantAction(() -> externTele.rsecondary.setPosition(0.22)),
                    new InstantAction(() -> externTele.primary.setPosition(0.53)),
                      new InstantAction(() -> externTele.rotation.setPosition(0.47))

            ));
        }
        //transfer
        if(gamepad1.dpad_left){
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> externTele.claw.setPosition(0.72)),
                    new InstantAction(() -> externTele.lext.setPosition(0.07)),
                    new InstantAction(() -> externTele.lext.setPosition(0.07))
            ));
        }


        // Declare a variable to track the current lift position state
         // Start at 0

// In your loop or function where the dpad_up action is checked
        if (gamepad1.dpad_up && !toggle) {  // Check if the button is pressed and toggle is false
            toggle = true;  // Prevents spamming, now the toggle is true
            // Change the lift position state based on the current state
            if (liftPositionState == 0) {
                liftPositionState = 1;  // Move to position 1
                lift.setTargetPosition(1000);
            } else if (liftPositionState == 1) {
                liftPositionState = 2;  // Move to position 2
                lift.setTargetPosition(1700);
            } else if (liftPositionState == 2) {
                liftPositionState = 0;  // Move back to position 0
                lift.setTargetPosition(0);
            }
        } else if (!gamepad1.dpad_up && toggle) {  // Button is released and toggle is true
            toggle = false;  // Reset toggle to allow future button presses
        }

        if(gamepad1.b){
            runningActions.add(new SequentialAction(
                    //turret.setTargetPosition(turret.getCurrentPosition()+ processor.calculateTurretAdjustment);
                    new InstantAction(() -> externTele.lext.setPosition(externTele.lext.getPosition()+processor.getExtensionAdjustment())),
                    new InstantAction(() -> externTele.rext.setPosition(externTele.rext.getPosition()+processor.getExtensionAdjustment())),
                    new InstantAction(() -> externTele.rotation.setPosition(externTele.rotation.getPosition()+processor.getServoAdjustment())),
                    new InstantAction(() -> externTele.lsecondary.setPosition(0.14)),
                    new InstantAction(() -> externTele.rsecondary.setPosition(0.14)),
                    new SleepAction(2.5),
                    new InstantAction(() -> externTele.claw.setPosition(0.5))

//                    new InstantAction(() -> externTele.rsecondary.setPosition(0.7)),
//                    //wait
//                    new InstantAction(() -> externTele.primary.setPosition(0.72)),
//                    new InstantAction(() -> externTele.rotation.setPosition(0.67))
                    //slide raise
                    //driver move bot forward to attach
            ));
        }

        ////score specimen
        if(gamepad1.dpad_down){
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> externTele.lsecondary.setPosition(0.16)),
                    new InstantAction(() -> externTele.rsecondary.setPosition(0.16)),
                    //wait
                    new InstantAction(() -> externTele.primary.setPosition(0.25)),
                    new InstantAction(() -> externTele.rotation.setPosition(0.47))
                    //slide raise
                    //driver move bot forward to attach
            ));
        }
        if (gamepad1.dpad_right && !dPadtoggle) {  // Check if the button is pressed and toggle is false
            dPadtoggle = true;  // Prevents spamming, now the toggle is true
            // Change the lift position state based on the current state
            if (exPositionState == 0) {
                exPositionState = 1;
                externTele.lext.setPosition(0); // Move to position when toggle is on
                externTele.rext.setPosition(0); // Move to position when toggle is on
            } else if (exPositionState == 1) {
                exPositionState = 0;
                externTele.lext.setPosition(0.25); // Move to position when toggle is off
                externTele.rext.setPosition(0.25); // Move to position when toggle is off
            }
        } else if (!gamepad1.dpad_right && dPadtoggle) {  // Button is released and toggle is true
            dPadtoggle = false;  // Reset toggle to allow future button presses
        }


        if (gamepad1.left_bumper && !clawtoggle) {  // Check if the button is pressed and toggle is false
            clawtoggle = true;  // Prevents spamming, now the toggle is true
            // Change the lift position state based on the current state
            if (clawPositionState == 0) {
                clawPositionState = 1;
                externTele.claw.setPosition(0.15);  // Close the claw
            } else if (clawPositionState == 1) {
                clawPositionState = 0;
                externTele.claw.setPosition(0.72);  // Open the claw
            }
        } else if (!gamepad1.left_bumper && clawtoggle) {  // Button is released and toggle is true
            clawtoggle = false;  // Reset toggle to allow future button presses
        }



        //make trigger change the rotation as you hold, left trigger rotate left, right trigger rotate right



        //
        if(gamepad1.a){
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> externTele.lext.setPosition(0.15)),
                    new InstantAction(() -> externTele.rext.setPosition(0.15)),
                    new InstantAction(() -> externTele.lsecondary.setPosition(0.22)),
                    new InstantAction(() -> externTele.rsecondary.setPosition(0.22)),
                    new InstantAction(() -> externTele.primary.setPosition(0.9))


                        ));
                    }


        //drivetrain code
        //drive train
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        double drive_y = gamepad1.left_stick_y;
        double drive_x = gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        double theta = Math.atan2(drive_y, drive_x);
        double power = Math.hypot(drive_x,drive_y);
        double sin = Math.sin(theta-PI/4);
        double cos = Math.cos(theta-PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        frontLeftPower = power * cos/max + turn;
        frontRightPower = power * sin/max - turn;
        backLeftPower = power*sin/max + turn;
        backRightPower = power *cos/max - turn;
        if((power+Math.abs(turn))>1) {
            frontLeftPower /= power+turn;
            frontRightPower /= power+turn;
            backLeftPower /= power+turn;
            backRightPower /= power+turn;
        }
        List<Double> list = Arrays.asList(1.0, Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower));
        double maximum = Collections.max(list); // returns the greatest number
        fl.setPower((frontLeftPower / maximum) * 0.7); // set the max power to 0.6
        if(fl.isBusy()){fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
        fr.setPower((frontRightPower / maximum) * 0.7);
        if(fr.isBusy()){fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
        bl.setPower((backLeftPower / maximum) * 0.7);
        if(bl.isBusy()){bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
        br.setPower((backRightPower / maximum) * 0.7);
        if(br.isBusy()){br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}



        turret.update();
        lift.update();
    }
}