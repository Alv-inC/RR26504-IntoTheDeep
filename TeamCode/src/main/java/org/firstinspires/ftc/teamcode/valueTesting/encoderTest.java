package org.firstinspires.ftc.teamcode.valueTesting;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.Subsystems.cameraProcessor;

import org.opencv.core.Scalar;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
//import org.firstinspires.ftc.teamcode.teamcode.testing.Turret;


@Config
@Autonomous
public class encoderTest extends LinearOpMode {
    public static boolean go, rotAdjust, extAdjust, turretAdjust, specimenAdjust, slidesGo, turretGo, ADJUST, sway = false;
    boolean again = true;
    public static double pext, ppri, psec, pclaw, prot, ptrans, pout, slidespos, turretpos;
    public static double width, height = 0;
    public static double kp = 0;
    public static boolean MASK_TOGGLE = true;
    public static double rotpos = 0.5;
    public double turretCorrection;
    public static boolean go1, go2, go3, goTogether, goLift = false;
    public static boolean servo2Reverse = false;
    public static double pos1, pos2, pos3, liftpos = 0.5;
    public Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        pext = 0.15; ppri = 0.67; psec = 0.28; pclaw = 0.45; prot = 0.47; ptrans= 0.05; pout=0.67; slidespos=0; turretpos=0;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        waitForStart();

        while(opModeIsActive()){


            telemetry.addData("servo1 pos", motor1.getCurrentPosition());
            telemetry.addData("servo2 pos", motor2.getCurrentPosition());
            telemetry.update();
        }
    }

}

