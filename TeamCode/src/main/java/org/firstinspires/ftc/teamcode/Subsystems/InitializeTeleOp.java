package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class InitializeTeleOp {
    public Servo rotation, lext, rext, lsecondary, rsecondary, primary, claw;
    public MultipleTelemetry telemetry;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize telemetry and vision
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize servos
        rotation = hardwareMap.get(Servo.class, "rotation");
        rotation.setPosition(0.47);

        lext = hardwareMap.get(Servo.class, "lext");
        rext = hardwareMap.get(Servo.class, "rext");
        lext.setDirection(Servo.Direction.REVERSE);
        lext.setPosition(0.15);
        rext.setPosition(0.15);

        lsecondary = hardwareMap.get(Servo.class, "lsecondary");
        rsecondary = hardwareMap.get(Servo.class, "rsecondary");
        lsecondary.setDirection(Servo.Direction.REVERSE);
        lsecondary.setPosition(0.34);
        rsecondary.setPosition(0.34);

        primary = hardwareMap.get(Servo.class, "primary");
        primary.setPosition(0.33);

        claw = hardwareMap.get(Servo.class, "claw");
    }
    public void waitWithoutStoppingRobot(double milliseconds) {
        ElapsedTime timer = new ElapsedTime(); // Create a timer instance
        timer.reset(); // Reset the timer to start at 0

        while (timer.milliseconds() < milliseconds) {
            // Perform other tasks or keep the robot running smoothly
            telemetry.addData("Waiting", "%.2f seconds remaining", milliseconds - timer.seconds());
            telemetry.update();
        }
    }
}
