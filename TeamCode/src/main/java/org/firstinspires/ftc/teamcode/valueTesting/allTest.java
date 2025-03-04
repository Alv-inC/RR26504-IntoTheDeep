package org.firstinspires.ftc.teamcode.valueTesting;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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


@TeleOp
@Config
public class allTest extends LinearOpMode {
    public static boolean c0g, c1g, c2g, c3g, e3g, e4g, e5g = false;
    public static boolean go1, go2, go3, goTogether, goLift = false;
    public static boolean servo2Reverse = false;
    public static double pos1, pos2, pos3, liftpos = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo c0 = hardwareMap.get(Servo.class, "c0");
        Servo c1 = hardwareMap.get(Servo.class, "c1");
        Servo c2 = hardwareMap.get(Servo.class, "c2");
        Servo c3 = hardwareMap.get(Servo.class, "c3");
        Servo e3 = hardwareMap.get(Servo.class, "e3");
        Servo e4 = hardwareMap.get(Servo.class, "e4");
        Servo e5 = hardwareMap.get(Servo.class, "e5");


        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a) {
                c0g = false;
                c0.setPosition(c0.getPosition()+0.2);
            }
            if(gamepad1.b) {
                c1g = false;
                c1.setPosition(c1.getPosition()+0.2);
            }
            if(gamepad1.x) {
                c2g = false;
                c2.setPosition(c2.getPosition()+0.2);
            }
            if(gamepad1.y) {
                c3g = false;
                c3.setPosition(c3.getPosition()+0.2);
            }
            if(gamepad1.dpad_left) {
                e3g = false;
                e3.setPosition(e3.getPosition()+0.2);
            }
            if(gamepad1.dpad_right) {
                e4g = false;
                e4.setPosition(e4.getPosition()+0.2);
            }
            if(gamepad1.dpad_down) {
                e5g = false;
                e5.setPosition(e5.getPosition()+0.2);
            }
            if(goTogether){
                goTogether = false;
                c0.setPosition(0.5);
                c1.setPosition(0.5);
                c2.setPosition(0.5);
                c3.setPosition(0.5);
                e3.setPosition(0.5);
                e4.setPosition(0.5);
                e5.setPosition(0.5);

            }

            telemetry.addData("c0",c0.getPosition());
            telemetry.addData("c1",c1.getPosition());
            telemetry.addData("c2",c2.getPosition());
            telemetry.addData("c3",c3.getPosition());
            telemetry.addData("e3",e3.getPosition());
            telemetry.addData("e4",e4.getPosition());
            telemetry.addData("e5",e5.getPosition());
            telemetry.update();
        }
    }

}

