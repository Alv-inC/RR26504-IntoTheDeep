package org.firstinspires.ftc.teamcode.Tests;


import static java.lang.Math.PI;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;
import java.util.Collections;
import java.util.concurrent.atomic.AtomicReference;
import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name="theteleop")
public class OpenCVTEST extends LinearOpMode {
    public static boolean clawgo, rotationgo, extensiongo, secondarygo = false;
    public static double width, height = 0;
    public static boolean MASK_TOGGLE = true;
    public static Scalar RANGE_LOW = new Scalar(170, 100, 0, 0);
    public static Scalar RANGE_HIGH = new Scalar(255, 255, 230, 255);
    public static double rotpos = 0.5;

    //Talk Volatile vs AtomicReference, final to make sure it is only initialized once, etc.
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
            if(largestContour != null){
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
            double offsetY = contourY - 480/2;
            double offsetMM = offsetY*152/480;
            double offsetTicks = offsetMM*0.33/303;
            offsetTicks *= -1;
            offsetTicks /= 2;
            offsetTicks += 0.03;
            if(offsetTicks< 0) offsetTicks -= 0.01
                    ;
            //else offsetTicks += 0.03;
            return offsetTicks;
        }


//        public double calculateTurretAdjustment(MatOfPoint contour) {
//            // Calculate the pixel offset from the center
//            double offsetY = contourCenterY - CENTER_Y;
//
//            // Convert pixel offset to required slide extension in millimeters
//            double targetExtensionMM = offsetY / PIXELS_PER_MM;
//
//            // Clamp the extension to the valid slide range
//            targetExtensionMM = Math.max(SLIDE_MIN_EXTENSION_MM,
//                    Math.min(SLIDE_MAX_EXTENSION_MM, targetExtensionMM));
//
//            // Map the slide extension in mm to servo positions (0.0 - 1.0)
//            return map(targetExtensionMM,
//                    SLIDE_MIN_EXTENSION_MM,
//                    SLIDE_MAX_EXTENSION_MM,
//                    FULL_RETRACTION_POSITION,
//                    FULL_EXTENSION_POSITION);
//
//        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize camera
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final CameraStreamProcessor processor = new CameraStreamProcessor();

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
        //initializeservos
        Servo rotation = hardwareMap.get(Servo.class, "rotation");
        rotation.setPosition(0.47);
        Servo lext = hardwareMap.get(Servo.class, "lext");
        Servo rext = hardwareMap.get(Servo.class, "rext");
        lext.setDirection(Servo.Direction.REVERSE);
        waitWithoutStoppingRobot(300);
        lext.setPosition(0.15);
        rext.setPosition(0.15);
        Servo lsecondary = hardwareMap.get(Servo.class, "lsecondary");
        Servo rsecondary = hardwareMap.get(Servo.class, "rsecondary");
        lsecondary.setDirection(Servo.Direction.REVERSE);
        waitWithoutStoppingRobot(300);
        lsecondary.setPosition(0.34);
        rsecondary.setPosition(0.34);
        waitWithoutStoppingRobot(300);
        Servo primary = hardwareMap.get(Servo.class, "primary");
        primary.setPosition(0.33);
//        Servo outtake = hardwareMap.get(Servo.class, "outtake");
//        outtake.setPosition(0.67);
//        Servo transfer = hardwareMap.get(Servo.class, "transfer");
//        transfer.setPosition(0);
        Servo claw = hardwareMap.get(Servo.class, "claw");


//        //initialize motors
//        DcMotor fr, fl, br, bl, turret;
//        turret = hardwareMap.get(DcMotor.class, "turret");
//        fr = hardwareMap.get(DcMotor.class, "fr");
//        fl = hardwareMap.get(DcMotor.class, "fl");
//        br = hardwareMap.get(DcMotor.class, "br");
//        bl = hardwareMap.get(DcMotor.class, "bl");
//        fr.setDirection(DcMotorSimple.Direction.REVERSE);
//        br.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (MASK_TOGGLE) {
                FtcDashboard.getInstance().sendImage(processor.getMaskedFrameBitmap());
            } else {
                FtcDashboard.getInstance().sendImage(processor.getLastFrame());
            }

            if (extensiongo) {
                extensiongo = false;
                lext.setPosition(lext.getPosition()+processor.getExtensionAdjustment());
                rext.setPosition(rext.getPosition()+processor.getExtensionAdjustment());
            }
            if(rotationgo){
                rotationgo = false;
                rotation.setPosition(rotation.getPosition()+processor.getServoAdjustment());
            }
            if(secondarygo){
                secondarygo = false;
                lsecondary.setPosition(0.24);
                rsecondary.setPosition(0.24);
            }
            if(clawgo){
                clawgo = false;
                claw.setPosition(0.5);
            }

            //intake position
            if(gamepad1.x){
                lext.setPosition(0.15);
                rext.setPosition(0.15);
                waitWithoutStoppingRobot(300);
                lsecondary.setPosition(0.34);
                rsecondary.setPosition(0.34);
                waitWithoutStoppingRobot(300);
                primary.setPosition(0.7);
            }
            //execute intake
            if(gamepad1.a){
                //turret.setTargetPosition(turret.getCurrentPosition()+ processor.calculateTurretAdjustment);
                waitWithoutStoppingRobot(300);
                lext.setPosition(lext.getPosition()+processor.getExtensionAdjustment());
                rext.setPosition(rext.getPosition()+processor.getExtensionAdjustment());
                waitWithoutStoppingRobot(300);
                rotation.setPosition(rotation.getPosition()+processor.getServoAdjustment());
                waitWithoutStoppingRobot(300);
                lsecondary.setPosition(0.24);
                rsecondary.setPosition(0.24);
                waitWithoutStoppingRobot(300);
                claw.setPosition(0.5);
            }
            //retract to base position
            if(gamepad1.y){
                rotation.setPosition(0.75);
                //turret.setTargetPosition(turretPosition);
                waitWithoutStoppingRobot(300);
                lsecondary.setPosition(0.34);
                rsecondary.setPosition(0.34);
                waitWithoutStoppingRobot(300);
                primary.setPosition(0.33);
                waitWithoutStoppingRobot(300);
                lext.setPosition(0);
                rext.setPosition(0);
            }
            //transfer
            if(gamepad1.dpad_left){
                claw.setPosition(0.72);
                waitWithoutStoppingRobot(300);
                lext.setPosition(0.07);
                lext.setPosition(0.07);
            }
            //score basket
            if(gamepad1.dpad_up){
//                transfer.setPosition(0.7);
//                waitWithoutStoppingRobot(300);
//                outtake.setPosition(0.72);
//                waitWithoutStoppingRobot(300);
//                //outtake.setPosition(0.67);
//                waitWithoutStoppingRobot(300);
//                //transfer.setPosition(0);
            }
            //score specimen
            if(gamepad1.dpad_down){
                //slide raise


            }
//            if(gamepad1.right_trigger>0) turret.setPower(gamepad1.right_trigger);
//            else if(gamepad1.left_trigger>0) turret.setPower(-gamepad1.left_trigger);

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
//            fl.setPower((frontLeftPower / maximum) * 0.7); // set the max power to 0.6
//            if(fl.isBusy()){fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
//            fr.setPower((frontRightPower / maximum) * 0.7);
//            if(fr.isBusy()){fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
//            bl.setPower((backLeftPower / maximum) * 0.7);
//            if(bl.isBusy()){bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
//            br.setPower((backRightPower / maximum) * 0.7);
//            if(br.isBusy()){br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}


            telemetry.addData("Servo adjustment: ", processor.getServoAdjustment());
            telemetry.addData("extension adjustment: ", processor.getExtensionAdjustment());
            telemetry.update();

            sleep(10);
        }
    }
    public void waitWithoutStoppingRobot(double milliseconds) {
        ElapsedTime timer = new ElapsedTime(); // Create a timer instance
        timer.reset(); // Reset the timer to start at 0

        while (timer.milliseconds() < milliseconds && opModeIsActive()) {
            // Perform other tasks or keep the robot running smoothly
            telemetry.addData("Waiting", "%.2f seconds remaining", milliseconds - timer.seconds());
            telemetry.update();
        }
    }

}