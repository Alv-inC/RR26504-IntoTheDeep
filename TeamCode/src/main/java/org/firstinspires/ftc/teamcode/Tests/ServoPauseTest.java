package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ServoPauseTest extends LinearOpMode {

    // Declare servo
    private Servo testServo;

    // Servo positions
    private static final double SERVO_POSITION_MIN = 0.0;
    private static final double SERVO_POSITION_MAX = 1.0;

    @Override
    public void runOpMode() {
        // Initialize the servo
        testServo = hardwareMap.get(Servo.class, "testServo");

        // Set initial position
        testServo.setPosition(SERVO_POSITION_MIN);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Check gamepad inputs to move servo
            if (gamepad1.a) {
                moveServoWithWait(testServo, SERVO_POSITION_MIN, 1); // Move to min and wait 1 second
            } else if (gamepad1.b) {
                moveServoWithWait(testServo, SERVO_POSITION_MAX, 2); // Move to max and wait 2 seconds
            }

            // Display servo position on telemetry
            telemetry.addData("Servo Position", testServo.getPosition());
            telemetry.update();
        }
    }

    /**
     * Moves a servo to a specified position and waits for a given amount of time.
     *
     * @param servo          The servo to move.
     * @param position       The target position (0.0 to 1.0).
     * @param waitTimeInSec  The time to wait after moving the servo, in seconds.
     */
    private void moveServoWithWait(Servo servo, double position, double waitTimeInSec) {
        // Set the servo to the desired position
        servo.setPosition(position);

        // Create a timer to manage the wait
        ElapsedTime timer = new ElapsedTime();

        // Wait for the specified time while the op mode is active
        while (timer.seconds() < waitTimeInSec && opModeIsActive()) {
            // Do nothing, just wait
        }
    }
}
