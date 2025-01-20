package org.firstinspires.ftc.teamcode.valueTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.Subsystems.cameraProcessor;
import org.opencv.core.Scalar;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;


@Config
@Autonomous
public class valueTesting extends LinearOpMode {
    public static boolean go, rotAdjust, extAdjust, turretAdjust = false;
    public static double pext, ppri, psec, pclaw, prot, ptrans, pout = 0.5;
    public static double width, height = 0;
    public static boolean MASK_TOGGLE = true;
    public static double rotpos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        final cameraProcessor processor = new cameraProcessor();
        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        Turret turret = new Turret(hardwareMap);
        Servo lext = hardwareMap.get(Servo.class, "lext");
        Servo rext = hardwareMap.get(Servo.class, "rext");
        lext.setDirection(Servo.Direction.REVERSE);
//
        Servo lsecondary = hardwareMap.get(Servo.class, "lsecondary");
        Servo rsecondary = hardwareMap.get(Servo.class, "rsecondary");
        lsecondary.setDirection(Servo.Direction.REVERSE);

        Servo rotation = hardwareMap.get(Servo.class, "rotation");
        Servo primary = hardwareMap.get(Servo.class, "primary");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo ltransfer = hardwareMap.get(Servo.class, "ltransfer");
        Servo rtransfer = hardwareMap.get(Servo.class, "rtransfer");
        ltransfer.setDirection(Servo.Direction.REVERSE);
//        Servo outtake = hardwareMap.get(Servo.class, "outtake");
        //Servo servo = hardwareMap.get(Servo.class, "primary");

        waitForStart();

        while(opModeIsActive()){
            if(go){
                go = false;
//                lext.setPosition(pext);
//                rext.setPosition(pext);
//                lsecondary.setPosition(psec);
//                rsecondary.setPosition(psec);
//                primary.setPosition(ppri);
                claw.setPosition(pclaw);
//                rotation.setPosition(prot);
//                ltransfer.setPosition(ptrans);
//                rtransfer.setPosition(ptrans);
//              outtake.setPosition(pout);
                //servo.setPosition(0.31);
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
//            if(turretAdjust){
//                turretAdjust = false;
//                turret.setTargetPosition();
//            }
            telemetry.clearAll();
//            telemetry.addData("transfer position", ltransfer.getPosition());
//            telemetry.addData("outtake position", outtake.getPosition());
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
}
