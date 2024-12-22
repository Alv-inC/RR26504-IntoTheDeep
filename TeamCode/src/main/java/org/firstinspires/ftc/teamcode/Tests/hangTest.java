package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class hangTest extends OpMode {
    private DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        // Check button inputs
        if (gamepad1.a) {
            motor.setPower(1.0); // Move motor forward
        } else if (gamepad1.b) {
            motor.setPower(-1.0); // Move motor backward
        } else {
            motor.setPower(0.0); // Stop motor
        }


        telemetry.addData("Motor Power", motor.getPower());
        telemetry.update();
    }
}
