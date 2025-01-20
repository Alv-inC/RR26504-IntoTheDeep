package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
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
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class cameraProcessor implements VisionProcessor, CameraStreamSource {
    public static double width, height = 0;
    public static boolean MASK_TOGGLE = true;
    public static Scalar RANGE_LOW = new Scalar(170, 100, 0, 0);
    public static Scalar RANGE_HIGH = new Scalar(255, 255, 230, 255);
    public static double rotpos = 0.5;
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

//    public double TurretAdjustment(MatOfPoint contour) {
//        // Calculate the pixel offset from the center
//        double offsetY = contourCenterY - CENTER_Y;
//
//        // Convert pixel offset to required slide extension in millimeters
//        double targetExtensionMM = offsetY / PIXELS_PER_MM;
//
//        // Clamp the extension to the valid slide range
//        targetExtensionMM = Math.max(SLIDE_MIN_EXTENSION_MM,
//                Math.min(SLIDE_MAX_EXTENSION_MM, targetExtensionMM));
//
//        // Map the slide extension in mm to servo positions (0.0 - 1.0)
//        return map(targetExtensionMM,
//                SLIDE_MIN_EXTENSION_MM,
//                SLIDE_MAX_EXTENSION_MM,
//                FULL_RETRACTION_POSITION,
//                FULL_EXTENSION_POSITION);
//
//    }
}
