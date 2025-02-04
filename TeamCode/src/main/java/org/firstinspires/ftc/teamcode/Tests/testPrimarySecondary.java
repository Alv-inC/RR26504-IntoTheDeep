package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SECONDARYPRIMARY", group = "Testing")
public class testPrimarySecondary extends OpMode {
    private Servo lsecondary;
    private Servo rsecondary;
    private Servo primary;

    @Override
    public void init() {
        primary = hardwareMap.get(Servo.class, "primary");
        lsecondary = hardwareMap.get(Servo.class, "lsecondary");
        rsecondary = hardwareMap.get(Servo.class, "rsecondary");
        lsecondary.setDirection(Servo.Direction.REVERSE);
        lsecondary.setPosition(0.34);
        rsecondary.setPosition(0.34);
        primary.setPosition(0.33);
    }

    @Override
    public void loop() {

    }
}
