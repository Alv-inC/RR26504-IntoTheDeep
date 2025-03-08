package org.firstinspires.ftc.teamcode.valueTesting;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
import org.firstinspires.ftc.teamcode.Subsystems.ChainActions;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.Subsystems.cameraProcessor;

import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
//import org.firstinspires.ftc.teamcode.teamcode.testing.Turret;


@Config
@TeleOp
public class servoTest extends LinearOpMode {
    public static boolean go1, go2, go3, go4, go5, turretGo, goTogether, goLift, ADJUST, adjPosition, goExt = false;
    public static boolean servo2Reverse = false;
    public static double pos1, pos2, pos3, pos4, pos5, liftpos, turretpos, posExt = 0.5;
    public Lift lift;
    Turret turret;
    cameraProcessor processor = new cameraProcessor(new Scalar(100, 0, 0,0), new Scalar(255, 5, 230,255), false);
    private List<Action> runningActions = new ArrayList<>();
    ChainActions chain;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo servo1 = hardwareMap.get(Servo.class, "rsecondary");
        Servo servo2 = hardwareMap.get(Servo.class, "lsecondary");
                servo2.setDirection(Servo.Direction.REVERSE);
        Servo servo3 = hardwareMap.get(Servo.class, "primary");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo rotation = hardwareMap.get(Servo.class, "rotation");

        Servo lext = hardwareMap.get(Servo.class, "lext");
        Servo rext = hardwareMap.get(Servo.class, "rext");
        rext.setDirection(Servo.Direction.REVERSE);
        lift = new Lift(hardwareMap, 1.5);
        turret = new Turret(hardwareMap, 0.5);
        chain = new ChainActions(hardwareMap);

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
        waitForStart();

        while(opModeIsActive()){
            FtcDashboard.getInstance().sendImage(processor.getMaskedFrameBitmap());
            TelemetryPacket packet = new TelemetryPacket();
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            if(go1) {
                go1 = false;
                servo1.setPosition(pos1);
            }
            if(go2) {
                go2 = false;
                servo2.setPosition(pos2);
            }
            if(goExt){
                goExt = false;
                lext.setPosition(posExt);
                rext.setPosition(posExt);
            }
            if(go3) {
                go3 = false;
                servo3.setPosition(pos3);
            }
            if(go4){
                go4 = false;
                claw.setPosition(pos4);
            }
            if(go5){
                go5 = false;
                rotation.setPosition(pos5);
            }
            if(goLift) {
                goLift = false;
                lift.setTargetPosition(liftpos);
            }
            if(goTogether){
                goTogether = false;
                servo1.setPosition(pos1);
                servo2.setPosition(pos2);
            }
            if(turretGo){
                turretGo = false;
                turret.setTargetPosition(turretpos);
            }
            if(adjPosition){
                adjPosition = false;
                runningActions.add(chain.intakePosition());
            }
            double rotAdjust = processor.getServoAdjustment();
            double turAdjust = processor.getTurretAdjustment();
            double extAdjust = processor.getExtensionAdjustment();
            if(ADJUST){
                ADJUST = false;
                //turret.setTargetPosition(turAdjust);
                runningActions.add(chain.intake(processor, false));
            }
            telemetry.addData("rot", rotAdjust);
            telemetry.addData("ext", extAdjust);
            telemetry.addData("tur", turAdjust);
            telemetry.addData("pos1", claw.getPosition());

                telemetry.update();
            lift.update();
            turret.update();
        }
    }

}

