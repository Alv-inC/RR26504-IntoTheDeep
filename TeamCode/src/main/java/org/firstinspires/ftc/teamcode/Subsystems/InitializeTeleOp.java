package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class InitializeTeleOp {
    public Servo rotation, lext, rext, lsecondary, rsecondary, primary, claw, trapdoor, ltransfer, rtransfer;
    public DcMotorEx hang;
    public MultipleTelemetry telemetry;

    public void initialize(HardwareMap hardwareMap) {



        //initialize motor
        hang = hardwareMap.get(DcMotorEx.class, "hang");
        // Initialize servos
        rotation = hardwareMap.get(Servo.class, "rotation");
        rotation.setPosition(0.47);

        lsecondary = hardwareMap.get(Servo.class, "lsecondary");
        rsecondary = hardwareMap.get(Servo.class, "rsecondary");
        lsecondary.setDirection(Servo.Direction.REVERSE);
        lsecondary.setPosition(0.27);
        rsecondary.setPosition(0.27);

        primary = hardwareMap.get(Servo.class, "primary");
        primary.setPosition(0);

        lext = hardwareMap.get(Servo.class, "lext");
        rext = hardwareMap.get(Servo.class, "rext");
        rext.setDirection(Servo.Direction.REVERSE);
        lext.setPosition(0.05);
        rext.setPosition(0.05);

        claw = hardwareMap.get(Servo.class, "claw");
    }
    public void waitWithoutStoppingRobotcc(double milliseconds) {
        ElapsedTime timer = new ElapsedTime(); // Create a timer instance
        timer.reset(); // Reset the timer to start at 0


    }
}
