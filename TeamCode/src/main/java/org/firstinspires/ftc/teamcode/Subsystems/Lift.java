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
public class Lift {
        private static PIDController liftPID;
        CachingDcMotorEx lift1;
        CachingDcMotorEx lift2;


        public static double p = 0.006, i = 0, d = 0;
        public static double targetPosition;
        private Telemetry telemetry;

        public static double liftAngle = 32;
        public static double liftMass = 0.3;
        public static double gravity = 9.81;
        public static double armLength = 0.4;
        public static double feedforwardScale = 0.3;
        //may need to increase


    //the max is 2700
        public Lift(HardwareMap hardwareMap){
            liftPID = new PIDController(p, i, d);
            liftPID.setPID(p,i,d);
            lift1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "lift1"), 0.005);
            lift2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "lift2"), 0.005);
            lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift1.setDirection(DcMotorSimple.Direction.REVERSE);
            //this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }

        public void update(){
            liftPID.setPID(p, i, d);
            int pos = lift1.getCurrentPosition();

            double power = liftPID.calculate(pos, targetPosition);

            double angleRadians = Math.toRadians(liftAngle);
            double feedforward = feedforwardScale * (liftMass * gravity * armLength * Math.cos(angleRadians));


            //to tune the PID
            // Send telemetry data
//            telemetry.addData("Target Position", targetPosition);
//            telemetry.addData("Current Position", pos);
//            telemetry.update();
            power = Math.max(-0.8, Math.min(0.8, power));
            lift1.setPower(power + feedforward);
            lift2.setPower(power + feedforward);
        }

        public void setTargetPosition(double targetPosition){
            this.targetPosition = targetPosition;
        }

        public boolean isClose(){
            return Math.abs(lift1.getCurrentPosition() - targetPosition) < 45;
        }

        public double getPosition(){

            return lift1.getCurrentPosition();
        }
        //

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
