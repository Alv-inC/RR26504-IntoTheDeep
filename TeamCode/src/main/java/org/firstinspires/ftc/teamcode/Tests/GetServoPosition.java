package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servo Tester", group = "Testing")
public class GetServoPosition extends OpMode {

    private Servo testServo1;
    private Servo testServo2;
    public static double position = 0.5; // Initial position for both servos, adjustable via dashboard
    private static final double POSITION_INCREMENT = 0.01; // Step size for adjustments

    private FtcDashboard dashboard;

    @Override
    public void init() {
        testServo1 = hardwareMap.get(Servo.class, "testServo1");
//        testServo2 = hardwareMap.get(Servo.class, "testServo2");


        //bottom position: servo1: 0.97, servo2: 0.03
        //top position: servp1: 0.3, servo2: 0.7
        dashboard = FtcDashboard.getInstance();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Increase position
        if (gamepad1.dpad_up) {
            position += POSITION_INCREMENT;
        }
        // Decrease position
        if (gamepad1.dpad_down) {
            position -= POSITION_INCREMENT;
        }

        // Clamp the position value between 0.0 and 1.0
        position = Math.max(0.0, Math.min(1.0, position));

        // Set the servo positions
        testServo1.setPosition(position);
        //
//        // Reverse the direction for testServo2
//        testServo2.setPosition(1.0 - position);

        // Send telemetry data
        telemetry.addData("Servo 1 Position", position);
//        telemetry.addData("Servo 2 Position", 1.0 - position);
        telemetry.update();

        // Send data to FTC Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Servo 1 Position", position);
//        packet.put("Servo 2 Position", 1.0 - position);
        dashboard.sendTelemetryPacket(packet);
    }
}
