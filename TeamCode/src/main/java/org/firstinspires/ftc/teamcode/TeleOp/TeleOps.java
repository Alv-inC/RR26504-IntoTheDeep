
package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@TeleOp (name="DriveTime!")
public class TeleOps extends OpMode {
    private DcMotor fr, fl, br, bl;
    @Override
    public void init() {
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
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
        backRightPower = power* cos/max - turn;

        if((power+Math.abs(turn))>1) {
            frontLeftPower /= power+turn;
            frontRightPower /= power+turn;
            backLeftPower /= power+turn;
            backRightPower /= power+turn;
        }


        // store the values in a list of doubles
        List<Double> list = Arrays.asList(1.0, Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower));
// //

        // get the greatest value
        double maximum = Collections.max(list); // returns the greatest number

//         set the powers
        fl.setPower((frontLeftPower / maximum) * 0.7); // set the max power to 0.6
        if(fl.isBusy()){fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
        fr.setPower((frontRightPower / maximum) * 0.7);
        if(fr.isBusy()){fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
        bl.setPower((backLeftPower / maximum) * 0.7);
        if(bl.isBusy()){bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
        br.setPower((backRightPower / maximum) * 0.7);
        if(br.isBusy()){br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}

    }
}
