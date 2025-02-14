package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class movementTest extends LinearOpMode{
    public static boolean strafe, rotate, go = false;
    public static double strafeTicks, rotateTicks, goTicks;

    MecanumDrive drive;
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();

        while(opModeIsActive()){
            if(strafe){
                strafe = false;
                strafe(drive.leftFront, drive.rightFront, drive.leftBack, drive.rightBack, strafeTicks);
            }
            if(go){
                go = false;
                go(drive.leftFront, drive.rightFront, drive.leftBack, drive.rightBack, goTicks);
            }
            if(rotate){
                rotate = false;
                rotate(drive.leftFront, drive.rightFront, drive.leftBack, drive.rightBack, rotateTicks);
            }
            telemetry.addData("leftFront encoder: ", drive.leftFront.getCurrentPosition());
            telemetry.addData("rightFront encoder: ", drive.rightFront.getCurrentPosition());
            telemetry.addData("leftBack encoder: ", drive.leftBack.getCurrentPosition());
            telemetry.update();
        }

    }
    private void strafe(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, double targetTicks){
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(targetTicks<0) {
            while (Math.abs(backLeft.getCurrentPosition()) < Math.abs(targetTicks)) {
                frontLeft.setPower(0.6);
                frontRight.setPower(-0.6);
                backLeft.setPower(-0.6);
                backRight.setPower(0.6);
            }
        } else{
            while (Math.abs(backLeft.getCurrentPosition()) < Math.abs(targetTicks)) {
                frontLeft.setPower(-0.6);
                frontRight.setPower(0.6);
                backLeft.setPower(0.6);
                backRight.setPower(-0.6);
            }
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    private void go(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, double targetTicks){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(targetTicks<0) {
            while (Math.abs(frontLeft.getCurrentPosition()) < Math.abs(targetTicks)) {
                frontLeft.setPower(-0.6);
                frontRight.setPower(-0.6);
                backLeft.setPower(-0.6);
                backRight.setPower(-0.6);
            }
        } else{
            while (Math.abs(frontLeft.getCurrentPosition()) < Math.abs(targetTicks)) {
                frontLeft.setPower(0.6);
                frontRight.setPower(0.6);
                backLeft.setPower(0.6);
                backRight.setPower(0.6);
            }
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    private void rotate(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, double targetTicks){
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(targetTicks<0) {
            while (Math.abs(frontLeft.getCurrentPosition()) < Math.abs(targetTicks)) {
                frontLeft.setPower(-0.6);
                frontRight.setPower(0.6);
                backLeft.setPower(-0.6);
                backRight.setPower(0.6);
            }
        } else{
            while (Math.abs(frontLeft.getCurrentPosition()) < Math.abs(targetTicks)) {
                frontLeft.setPower(0.6);
                frontRight.setPower(-0.6);
                backLeft.setPower(0.6);
                backRight.setPower(-0.6);
            }
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
