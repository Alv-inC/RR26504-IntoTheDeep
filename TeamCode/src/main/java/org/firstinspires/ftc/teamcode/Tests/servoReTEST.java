package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SERVORETEST", group = "Testing")
public class servoReTEST {
    private final Map<Servo, Double> initialPositions = new HashMap<>();
    private final Map<Servo, Boolean> toggleStates = new HashMap<>();
    private final Map<Servo, Boolean> buttonStates = new HashMap<>();

    private Servo rotation, lext, rext, lsecondary, rsecondary, primary, claw;

    public void initialize(HardwareMap hardwareMap) {
        rotation = hardwareMap.get(Servo.class, "rotation");
        lext = hardwareMap.get(Servo.class, "lext");
        rext = hardwareMap.get(Servo.class, "rext");
        lsecondary = hardwareMap.get(Servo.class, "lsecondary");
        rsecondary = hardwareMap.get(Servo.class, "rsecondary");
        primary = hardwareMap.get(Servo.class, "primary");
        claw = hardwareMap.get(Servo.class, "claw");

        lext.setDirection(Servo.Direction.REVERSE);
        lsecondary.setDirection(Servo.Direction.REVERSE);

        setInitialPosition(rotation, 0.47);
        setInitialPosition(lext, 0.15);
        setInitialPosition(rext, 0.15);
        setInitialPosition(lsecondary, 0.34);
        setInitialPosition(rsecondary, 0.34);
        setInitialPosition(primary, 0.33);
    }

    private void setInitialPosition(Servo servo, double position) {
        servo.setPosition(position);
        initialPositions.put(servo, position);
        toggleStates.put(servo, false);
        buttonStates.put(servo, false);
    }

    public void update(Gamepad gamepad) {
        handleToggle(gamepad.a, rotation);
        handleToggle(gamepad.b, lext);
        handleToggle(gamepad.x, rext);
        handleToggle(gamepad.y, lsecondary);
        handleToggle(gamepad.left_bumper, rsecondary);
        handleToggle(gamepad.right_bumper, primary);
        handleToggle(gamepad.dpad_up, claw);
    }

    private void handleToggle(boolean buttonPressed, Servo servo) {
        if (buttonPressed && !buttonStates.get(servo)) {
            toggleStates.put(servo, !toggleStates.get(servo));
            servo.setPosition(toggleStates.get(servo) ? 0.0 : initialPositions.get(servo));
        }
        buttonStates.put(servo, buttonPressed);
    }
}
