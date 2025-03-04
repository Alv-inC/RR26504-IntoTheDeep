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
public class valueTesting extends LinearOpMode {
    public static boolean go, rotAdjust, extAdjust, turretAdjust, specimenAdjust, slidesGo, turretGo, ADJUST, newTEST, sway = false;
    boolean again = true;
    public static double pext, ppri, psec, pclaw, prot, ptrans, pout, slidespos, turretpos, AAA;
    public static double width, height = 0;
    public static double kp = 0;
    public static boolean MASK_TOGGLE = true;
    public static double rotpos = 0.5;
    public double turretCorrection;

    @Override
    public void runOpMode() throws InterruptedException {
        pext = 0.15; ppri = 0.67; psec = 0.28; pclaw = 0.45; prot = 0.47; ptrans= 0.05; pout=0.67; slidespos=0; turretpos=0;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final cameraProcessor processor = new cameraProcessor(new Scalar(100, 0, 0,0), new Scalar(255, 5, 230,255), false);
        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        Turret turret = new Turret(hardwareMap);
        Servo lext = hardwareMap.get(Servo.class, "lext");
        Servo rext = hardwareMap.get(Servo.class, "rext");
        lext.setDirection(Servo.Direction.REVERSE);

        Servo lsecondary = hardwareMap.get(Servo.class, "lsecondary");
        Servo rsecondary = hardwareMap.get(Servo.class, "rsecondary");
        lsecondary.setDirection(Servo.Direction.REVERSE);

        Servo rotation = hardwareMap.get(Servo.class, "rotation");
        Servo primary = hardwareMap.get(Servo.class, "primary");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.REVERSE);
        Lift lift = new Lift(hardwareMap);
        DcMotor fr, fl, br, bl;
        fr = hardwareMap.get(DcMotor.class, "rightFront");
        fl = hardwareMap.get(DcMotor.class, "leftFront");
        br = hardwareMap.get(DcMotor.class, "rightBack");
        bl = hardwareMap.get(DcMotor.class, "leftBack");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            if (MASK_TOGGLE) {
                FtcDashboard.getInstance().sendImage(processor.getMaskedFrameBitmap());
            } else {
                FtcDashboard.getInstance().sendImage(processor.getLastFrame());
            }

            if(newTEST){
                newTEST = false;
                claw.setPosition(AAA);
            }
            if(go){
                go = false;
                lext.setPosition(pext);
                rext.setPosition(pext);
                lsecondary.setPosition(psec);
                rsecondary.setPosition(psec);
                primary.setPosition(ppri);
                claw.setPosition(pclaw);
                rotation.setPosition(prot);
            }
            if(turretGo){
                turretGo = false;
                turret.setTargetPosition(turretpos);
            }
            if(slidesGo){
                slidesGo = false;
                lift.setTargetPosition(slidespos);
            }
            if(rotAdjust){
                rotAdjust = false;
                rotation.setPosition(rotation.getPosition()+processor.getServoAdjustment());
            }
            if(extAdjust){
                extAdjust = false;
                lext.setPosition(lext.getPosition()+processor.getExtensionAdjustment());
                rext.setPosition(lext.getPosition()+processor.getExtensionAdjustment());
            }
            if(turretAdjust){
                turretAdjust = false;
                turret.setTargetPosition(turret.getCurrentPosition()+processor.getTurretAdjustment());
            }
            if(sway){
                while(turret.getCurrentPosition()<300){
                    turret.setPower(0.4);
                    if(processor.getArea()>=500){
                        again = false;
                        turret.setTargetPosition(turret.getCurrentPosition());
                        break;
                    }
                }
                if(again){
                    while(turret.getCurrentPosition()>-300){
                        turret.setPower(-0.4);
                        if(processor.getArea()>=500){
                            turret.setTargetPosition(turret.getCurrentPosition());
                            break;
                        }
                    }
                }
            }
            if(specimenAdjust){
                specimenAdjust = false;
                strafe(fl, fr, bl, br, processor.getSpecimenAdjustment());
            }
            if(ADJUST){
                ADJUST = false;
                turret.setTargetPosition(turret.getCurrentPosition()+processor.getTurretAdjustment());
                waitWithoutStoppingRobot(500);
                lext.setPosition(lext.getPosition()+processor.getExtensionAdjustment());
                rext.setPosition(lext.getPosition()+processor.getExtensionAdjustment());
                waitWithoutStoppingRobot(500);
                rotation.setPosition(rotation.getPosition()+processor.getServoAdjustment());
                waitWithoutStoppingRobot(300);
                lsecondary.setPosition(0.18);
                rsecondary.setPosition(0.18);
                waitWithoutStoppingRobot(300);
                claw.setPosition(0.55);
            }
            boolean previousButtonState = false; // Stores the previous state of the button

            if (gamepad1.a && !previousButtonState) {
                double adj = processor.getTurretAdjustment();
                turret.setTargetPosition(turret.getCurrentPosition()+processor.getTurretAdjustment());
                turretCorrection += adj;
                waitWithoutStoppingRobot(300);
                adj = processor.getTurretAdjustment();
                if(Math.abs(adj)>30){
                    turret.setTargetPosition(turret.getCurrentPosition()+processor.getTurretAdjustment());
                    turretCorrection += adj;
                }
                lext.setPosition(lext.getPosition()+processor.getExtensionAdjustment());
                rext.setPosition(lext.getPosition()+processor.getExtensionAdjustment());
                waitWithoutStoppingRobot(500);
                adj = processor.getExtensionAdjustment();
                if(Math.abs(processor.getExtensionAdjustment())>0.005) {
                    lext.setPosition(lext.getPosition()+adj);
                    rext.setPosition(lext.getPosition()+adj);
                    waitWithoutStoppingRobot(500);
                }
                rotation.setPosition(rotation.getPosition()+processor.getServoAdjustment());
                waitWithoutStoppingRobot(300);
                lsecondary.setPosition(0.16);
                rsecondary.setPosition(0.16);
                waitWithoutStoppingRobot(300);
                claw.setPosition(0.55);
            }
            previousButtonState = gamepad1.a;
            if(gamepad1.b){
                lext.setPosition(0.15);
                rext.setPosition(0.15);
                lsecondary.setPosition(0.28);
                rsecondary.setPosition(0.28);
                primary.setPosition(0.67);
                rotation.setPosition(0.47);
                turret.setTargetPosition(turret.getCurrentPosition()+processor.getTurretAdjustment());
                turretCorrection = 0;
            }
            if(gamepad1.y){
                primary.setPosition(0.3);
                lsecondary.setPosition(0.18);
                rsecondary.setPosition(0.18);
            }

            if(gamepad1.dpad_left){
                //move back
                turret.setTargetPosition(turret.getCurrentPosition()+processor.getTurretAdjustment());
                //move to zone
                strafe(fl, fr, bl, br, processor.getSpecimenAdjustment());
                waitWithoutStoppingRobot(500);
                //move to specimen
                waitWithoutStoppingRobot(500);

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
            if(gamepad1.left_bumper) claw.setPosition(0.45);
            else if(gamepad1.right_bumper)claw.setPosition(0.55);
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

            telemetry.addData("turret adjustment", processor.getTurretAdjustment());
            telemetry.addData("turret position", turret.getCurrentPosition());
            telemetry.addData("rotation adjustment", processor.getServoAdjustment());
            telemetry.addData("rotation position", rotation.getPosition());
            telemetry.addData("slides adjustment", processor.getExtensionAdjustment());
            telemetry.addData("slides position", lext.getPosition());
            telemetry.addData("specimen adjustment", processor.getSpecimenAdjustment());
            telemetry.addData("odometry position", bl.getCurrentPosition());
            telemetry.addData("odometry position", bl.getCurrentPosition());

            lift.update();
            turret.update();
            telemetry.update();
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
    private void strafe(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, double targetTicks){
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(targetTicks<0) {
            // Wait until movement finishes
            while (opModeIsActive() && Math.abs(backLeft.getCurrentPosition()) < Math.abs(targetTicks)) {
                // Keep looping
                frontLeft.setPower(0.6);
                frontRight.setPower(-0.6);
                backLeft.setPower(-0.6);
                backRight.setPower(0.6);
                telemetry.addLine("Strafing...");
                telemetry.addData("Current Position", backLeft.getCurrentPosition());
                telemetry.addData("Target Position", targetTicks);
                telemetry.update();
            }
        } else{
            while (opModeIsActive() && Math.abs(backLeft.getCurrentPosition()) < Math.abs(targetTicks)) {
                // Keep looping
                frontLeft.setPower(-0.6);
                frontRight.setPower(0.6);
                backLeft.setPower(0.6);
                backRight.setPower(-0.6);
                telemetry.addLine("Strafing...");
                telemetry.addData("Current Position", backLeft.getCurrentPosition());
                telemetry.addData("Target Position", targetTicks);
                telemetry.update();
            }
        }

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

}

