package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Lift Tuner", group = "Testing")
public class individualLiftTest extends OpMode {

    public static int lift1Target = 0;  // Adjustable in FTC Dashboard
    public static int lift2Target = 0;  // Adjustable in FTC Dashboard
    public static double power = 0.5;   // Adjustable lift power

    private DcMotorEx lift1, lift2;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift1.setTargetPosition(0);
        lift2.setTargetPosition(0);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
    }

    @Override
    public void loop() {
        // Set new target positions
        lift1.setTargetPosition(lift1Target);
        lift2.setTargetPosition(lift2Target);

        // Set power to move the motors
        lift1.setPower(power);
        lift2.setPower(power);

        // Send telemetry to FTC Dashboard
        telemetry.addData("Lift1 Position", lift1.getCurrentPosition());
        telemetry.addData("Lift2 Position", lift2.getCurrentPosition());
        telemetry.addData("Lift1 Target", lift1Target);
        telemetry.addData("Lift2 Target", lift2Target);
        telemetry.addData("Power", power);
        telemetry.update();
    }

}
