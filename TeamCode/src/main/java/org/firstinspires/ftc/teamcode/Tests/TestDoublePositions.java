package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Dual Servo Position Tester", group = "Testing")
public class TestDoublePositions extends OpMode {

    private Servo testServo1;
    private Servo testServo2;

    // Bottom position
    private static final double BOTTOM_SERVO1 = 0.97;
    private static final double BOTTOM_SERVO2 = 0.03;

    // Top position
    private static final double TOP_SERVO1 = 0.3;
    private static final double TOP_SERVO2 = 0.7;

    private FtcDashboard dashboard;

    private boolean isAtTopPosition = false; // To track the current position (top or bottom)

    @Override
    public void init() {
        testServo1 = hardwareMap.get(Servo.class, "testServo1");
        testServo2 = hardwareMap.get(Servo.class, "testServo2");

        // Set initial positions (can be set to bottom or top based on your preference)
        testServo1.setPosition(BOTTOM_SERVO1);
        testServo2.setPosition(BOTTOM_SERVO2);

        dashboard = FtcDashboard.getInstance();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Check for gamepad up and down to toggle positions
        if (gamepad1.dpad_up) {
            if (!isAtTopPosition) {
                // Move to top position
                testServo1.setPosition(TOP_SERVO1);
                testServo2.setPosition(TOP_SERVO2);
                isAtTopPosition = true;
            }
        }

        if (gamepad1.dpad_down) {
            if (isAtTopPosition) {
                // Move to bottom position
                testServo1.setPosition(BOTTOM_SERVO1);
                testServo2.setPosition(BOTTOM_SERVO2);
                isAtTopPosition = false;
            }
        }

        // Send telemetry data
        telemetry.addData("Servo 1 Position", testServo1.getPosition());
        telemetry.addData("Servo 2 Position", testServo2.getPosition());
        telemetry.addData("Position", isAtTopPosition ? "Top" : "Bottom");
        telemetry.update();

        // Send data to FTC Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Servo 1 Position", testServo1.getPosition());
        packet.put("Servo 2 Position", testServo2.getPosition());
        packet.put("Position", isAtTopPosition ? "Top" : "Bottom");
        dashboard.sendTelemetryPacket(packet);
    }
}
