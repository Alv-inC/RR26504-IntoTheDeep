// Import necessary FTC libraries
package org.firstinspires.ftc.teamcode.Tests;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class clawTest extends OpMode {

    // Declare a servo object for the claw
    private Servo clawServo;

    // Declare claw state variables
    private boolean clawOpen = false;

    @Override
    public void init() {
        // Initialize the claw servo
        clawServo = hardwareMap.get(Servo.class, "claw");
    }

    @Override
    public void loop() {
        // Check if A button is pressed (to open the claw)
        if (gamepad1.a) {
            clawServo.setPosition(1);  // Claw open
            clawOpen = true;
        }

        // Check if B button is pressed (to close the claw)
        if (gamepad1.b) {
            clawServo.setPosition(0.6);  // Claw closed
            clawOpen = false;
        }

        // Telemetry for debugging
        telemetry.addData("Claw Open", clawOpen ? "Yes" : "No");
        telemetry.update();
    }
}
