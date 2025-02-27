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
public class servoTest extends LinearOpMode {
    public static boolean go1, go2, goTogether, goLift = false;
    public static boolean servo2Reverse = false;
    public static double pos1, pos2, liftpos = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //Lift lift = new Lift(hardwareMap);
        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");


        waitForStart();

        while(opModeIsActive()){
            if(servo2Reverse) {
                servo2Reverse = false;
                servo2.setDirection(Servo.Direction.REVERSE);
            }
            if(go1) {
                go1 = false;
                servo1.setPosition(pos1);
            }
            if(go2) {
                go2 = false;
                servo2.setPosition(pos2);
            }
            if(goLift) {
                goLift = false;
                //lift.setTargetPosition(liftpos);
            }
            if(goTogether){
                goTogether = false;
                servo1.setPosition(pos1);
                servo2.setPosition(pos2);
            }
            telemetry.addData("servo1 pos", servo1.getPosition());
            telemetry.addData("servo2 pos", servo2.getPosition());
            telemetry.update();
        }

    }


}

