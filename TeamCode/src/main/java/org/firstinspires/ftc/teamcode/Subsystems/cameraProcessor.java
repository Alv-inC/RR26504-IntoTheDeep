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
    public static Scalar RANGE_LOW_1, RANGE_HIGH_1, RANGE_LOW_2, RANGE_HIGH_2;
    public boolean yellow;
    Mat peopleMask = new Mat();
    private final AtomicReference<Double> servoAdjustment = new AtomicReference<>(0.0);
    private final AtomicReference<Double> extensionAdjustment = new AtomicReference<>(0.0);
    private final AtomicReference<Double> turretAdjustment = new AtomicReference<>(0.0);
    private final AtomicReference<Double> specimenAdjustment = new AtomicReference<>(0.0);
    private final AtomicReference<Boolean> inFrame = new AtomicReference<>(false);
    private final AtomicReference<Double> area = new AtomicReference<>(0.0);
    private final AtomicReference<Scalar> centerHSV = new AtomicReference<>(new Scalar(0, 0, 0, 0));
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private final AtomicReference<Bitmap> peopleMaskedFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public cameraProcessor(Scalar low, Scalar high, boolean yellow){
        this.yellow = yellow;
        RANGE_LOW_1 = low;
        RANGE_HIGH_1 = high;
    }
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
    public double getArea(){
        return area.get();
    }
    public boolean getInFrame(){
        return inFrame.get();
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
    public double getSpecimenAdjustment(){
        return specimenAdjustment.get();
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
        inFrame.set(false);
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

        if(!yellow){
            RANGE_LOW_2 = new Scalar(0, 0, 0, 0);
            RANGE_HIGH_2 = new Scalar(0, 0, 0, 0);
        } else{
            RANGE_LOW_2 = new Scalar(170, 100, 0,0);
            RANGE_HIGH_2 = new Scalar(255, 255, 230,255);
        }

        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat peopleMask = new Mat();
        Core.inRange(frame, RANGE_LOW_1, RANGE_HIGH_1, mask1);
        Core.inRange(frame, RANGE_LOW_2, RANGE_HIGH_2, mask2);
        Core.bitwise_or(mask1, mask2, peopleMask);

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
        area.set(maxArea);

        if(largestContour != null){
            inFrame.set(true);
            RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
            servoAdjustment.set(calculateServoAdjustment(rotatedRect));
            extensionAdjustment.set(calculateExtensionAdjustment(rotatedRect));
            turretAdjustment.set(calculateTurretAdjustment(rotatedRect));
            specimenAdjustment.set(calculateSpecimenAdjustment(rotatedRect));
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
        double y = rotatedRect.center.y;
        y -= 240;
        if(y<= 240) y *= -1;
        double result;
        result = (8*y)/240;
        result = (result*0.1)/17;
        double error = (5.4*0.1)/8;
        if(result<0) result += error;
        else result += error;
        return result > 0.155 ? 0.14 : result;
    }

    public double calculateTurretAdjustment(RotatedRect rotatedRect) {
        // Get the X-coordinate of the object's center
        double x = rotatedRect.center.x;
        x -= 320;
        if(x>= 320) x *= -1;
        x /= 4.5;
        double y = rotatedRect.center.y;
        y = y -= 480;
        y/=2;
        y *= -1;
        y = 80;
        double theta = Math.atan(x/(y*3));
        double arcLength = theta * 145;
        double result = ((arcLength * 1250) / 455*-1);
        return result <150 && result/1.3 > -150 ? result : 0;
    }

    public double calculateSpecimenAdjustment(RotatedRect rotatedRect) {
        // Get X offset in pixels
        double objectX = rotatedRect.center.x;
        double offsetX = objectX - (640 / 2);  // Camera center is at 320 px

        // Convert pixel offset to millimeters
        double offsetMM = (offsetX / 640) * 220; // 230mm is the actual frame width

        // Convert millimeters to encoder ticks
        double wheelCircumference = 110; // mm
        double ticksPerRevolution = 8192; // Example for GoBILDA 312 RPM motors
        double ticksPerMM = ticksPerRevolution / wheelCircumference;

        return (offsetMM * ticksPerMM)/2.8; // Returns encoder ticks needed to center object
    }
}
