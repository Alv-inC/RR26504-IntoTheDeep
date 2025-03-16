package org.firstinspires.ftc.teamcode.Tests;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "CHANSEY TIME!!!", group = "Concept")
public class colorFollowChansey extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private CRServo slides;

    private OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        slides = hardwareMap.crservo.get("slides");

        slides.setPower(0);

        // Reverse the right motors if necessary
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);

        webcam.setPipeline(new RedDetectionPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Camera failed to open with error code " + errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            // Control the slides with gamepad buttons
            if (gamepad1.x) {
                slides.setPower(1); // Move slides up
            } else if (gamepad1.a) {
                slides.setPower(0); // Stop slides
            } else if (gamepad1.b) {
                slides.setPower(-1); // Move slides down
            }

            sleep(50); // Small delay to prevent overloading the CPU
        }
    }

    class RedDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Convert the frame to HSV color space
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define the range for red color in HSV
            Scalar lowerRed = new Scalar(0, 120, 70);
            Scalar upperRed = new Scalar(10, 255, 255);
            Scalar lowerRed2 = new Scalar(170, 120, 70);
            Scalar upperRed2 = new Scalar(180, 255, 255);

            // Threshold the HSV image to get only red colors
            Mat redMask1 = new Mat();
            Mat redMask2 = new Mat();
            Core.inRange(hsv, lowerRed, upperRed, redMask1);
            Core.inRange(hsv, lowerRed2, upperRed2, redMask2);

            Mat redMask = new Mat();
            Core.bitwise_or(redMask1, redMask2, redMask);

            // Find contours of the red objects
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(redMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest contour (most prominent red object)
            double maxArea = 0;
            Rect boundingRect = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    boundingRect = Imgproc.boundingRect(contour);
                }
            }

            if (boundingRect != null) {
                // Calculate the center of the bounding box
                int centerX = boundingRect.x + boundingRect.width / 2;
                int centerY = boundingRect.y + boundingRect.height / 2;

                // Calculate errors for movement
                double errorX = centerX - 160; // Horizontal error (320 / 2 = 160)
                double errorY = 120 - centerY; // Vertical error (240 / 2 = 120)

                // Proportional control for smoother movement
                double strafePower = errorX * 0.005; // Adjust the multiplier for sensitivity
                double forwardPower = 0.3; // Constant forward power

                // Limit the power to avoid excessive speed
                strafePower = Math.max(-0.5, Math.min(0.5, strafePower));

                // Set motor powers for mecanum drive
                setMotorPowers(forwardPower - strafePower, forwardPower + strafePower,
                        forwardPower + strafePower, forwardPower - strafePower);
            } else {
                // No red object detected, stop the robot
                setMotorPowers(0, 0, 0, 0);
            }

            return input;
        }

        private void setMotorPowers(double frontLeft, double backLeft, double frontRight, double backRight) {
            frontLeftMotor.setPower(frontLeft);
            backLeftMotor.setPower(backLeft);
            frontRightMotor.setPower(frontRight);
            backRightMotor.setPower(backRight);
        }
    }
}