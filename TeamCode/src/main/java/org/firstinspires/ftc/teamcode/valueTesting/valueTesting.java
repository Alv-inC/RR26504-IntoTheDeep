package org.firstinspires.ftc.teamcode.valueTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.Subsystems.cameraProcessor;
import org.opencv.core.Scalar;
//import org.firstinspires.ftc.teamcode.teamcode.testing.Turret;


@Config
@Autonomous
public class valueTesting extends LinearOpMode {
    public static boolean go, rotAdjust, extAdjust, turretAdjust, ADJUST = false;
    public static double pext, ppri, psec, pclaw, prot, ptrans, pout;
    public static double width, height = 0;
    public static boolean MASK_TOGGLE = true;
    public static double rotpos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        pext = 0.15; ppri = 0.67; psec = 0.25; pclaw = 0.75; prot = 0.47;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final cameraProcessor processor = new cameraProcessor();
        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        DcMotor turret = hardwareMap.get(DcMotor.class, "turret");
        Servo lext = hardwareMap.get(Servo.class, "lext");
        Servo rext = hardwareMap.get(Servo.class, "rext");
        lext.setDirection(Servo.Direction.REVERSE);

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
            if (MASK_TOGGLE) {
                FtcDashboard.getInstance().sendImage(processor.getMaskedFrameBitmap());
            } else {
                FtcDashboard.getInstance().sendImage(processor.getLastFrame());
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
            if(turretAdjust){
                turretAdjust = false;
                moveMotorToPosition(turret, (int) (processor.getTurretAdjustment()), 0.5);
            }
            if(ADJUST){
                ADJUST = false;
                moveMotorToPosition(turret, (int) (processor.getTurretAdjustment()), 0.5);
                waitWithoutStoppingRobot(1000);
                moveMotorToPosition(turret, (int) (processor.getTurretAdjustment()), 0.5);
                waitWithoutStoppingRobot(1000);
                lext.setPosition(lext.getPosition()+processor.getExtensionAdjustment());
                rext.setPosition(lext.getPosition()+processor.getExtensionAdjustment());
                waitWithoutStoppingRobot(1000);
                rotation.setPosition(rotation.getPosition()+processor.getServoAdjustment());
                waitWithoutStoppingRobot(1000);
                lsecondary.setPosition(0.15);
                rsecondary.setPosition(0.15);
                waitWithoutStoppingRobot(1000);
                claw.setPosition(0.67);
            }
            telemetry.addData("turret adjustment", processor.getTurretAdjustment());
            telemetry.addData("turret position", turret.getCurrentPosition());
            telemetry.addData("rotation adjustment", processor.getServoAdjustment());
            telemetry.addData("rotation position", rotation.getPosition());
            telemetry.addData("slides adjustment", processor.getExtensionAdjustment());
            telemetry.addData("slides position", lext.getPosition());


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
    public void moveMotorToPosition(DcMotor motor, int targetPosition, double power) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
        motor.setTargetPosition(targetPosition);              // Set target position
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);       // Enable RUN_TO_POSITION mode
        motor.setPower(power);                                // Set motor power

        while (motor.isBusy() && opModeIsActive()) {
            // Wait until the motor reaches the target position
            telemetry.addData("Motor Position", "Current: %d, Target: %d", motor.getCurrentPosition(), targetPosition);
            telemetry.update();
        }

        motor.setPower(0); // Stop the motor
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Switch back to normal mode
    }
}
