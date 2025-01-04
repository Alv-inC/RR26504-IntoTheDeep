package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Control with Button", group = "TeleOp")
public class motorTest extends OpMode {

    private DcMotor motor; // Declare the motor

    @Override
    public void init() {
        // Initialize the motor
        motor = hardwareMap.get(DcMotor.class, "motor1"); // Replace "motor" with the name in your configuration
        motor.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Check if the button (e.g., 'A' button on gamepad1) is pressed
        if (gamepad1.a) {
            motor.setPower(1.0); // Set motor power to full speed
        } else {
            motor.setPower(0); // Stop the motor when the button is released
        }

        // Display motor status on telemetry
        telemetry.addData("Motor Power", motor.getPower());
        telemetry.update();
    }
}
