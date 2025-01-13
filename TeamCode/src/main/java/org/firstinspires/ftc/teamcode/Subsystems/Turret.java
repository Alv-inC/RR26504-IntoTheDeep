package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
public class Turret {
    private static PIDController turretPID;
    CachingDcMotorEx turretMotor;




    public static double p = 0.0098, i = 0, d = 0.0001;
    public static double targetPosition = 0;
    //0 is the start position
    //-1250 is the 180 position
    //negative is going clockwise direction
    private Telemetry telemetry;


    public Turret(HardwareMap hardwareMap, Telemetry telemetry){
        turretPID = new PIDController(p, i, d);
        turretPID.setPID(p,i,d);
        turretMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "turret"), 0.005);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void update(){
        turretPID.setPID(p, i, d);
        int pos = turretMotor.getCurrentPosition();

        double power = (turretPID.calculate(pos, targetPosition) / 2);

        //to tune the PID, test P-0.00#, D-0.0001
        // Send telemetry data
        telemetry.addData("Current Position", pos);
        telemetry.addData("Target Position", targetPosition);
        telemetry.update();

        turretMotor.setPower(power);

    }

    public void setTargetPosition(double targetPosition){
        this.targetPosition = targetPosition;
    }

    public boolean isClose(){
        return Math.abs(turretMotor.getCurrentPosition() - targetPosition) < 45;
    }


    public Action setTargetPositionAction(double targetPosition){
        return t -> {
            setTargetPosition(targetPosition);
            return false;
        };
    }

    public Action waitUntilDone(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket t){
                return !isClose();
            }
        };
    }
    public Action updateAction(){
        return t -> {
            update();
            return true;
        };
    }

}
