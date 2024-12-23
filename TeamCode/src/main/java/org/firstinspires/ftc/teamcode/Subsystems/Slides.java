package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

import java.util.List;

public class Slides{
    private final HardwareMap hardwareMap;
    private DcMotorEx slideL;
    private DcMotorEx slideR;
    public static int SLIDE_VELOCITY = 5000;
    public static int SLIDES_EXTENDED = 0;
    public static int SLIDES_RETRACTED = 0;
    public static int SLIDES_MAX = 0;

    public Slides(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }


    public void initialize(){
        // Bulk Read
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        slideL = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideR = hardwareMap.get(DcMotorEx.class, "slideRight");

// Config slides
        slideL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideL.setTargetPosition(slideL.getCurrentPosition());
        slideL.setMode(RunMode.RUN_TO_POSITION);
        slideL.setVelocity(SLIDE_VELOCITY);

        slideR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideR.setTargetPosition(slideL.getCurrentPosition());
        slideR.setMode(RunMode.RUN_TO_POSITION);
        slideR.setVelocity(SLIDE_VELOCITY);


        //may need to change this
        slideL.setDirection(DcMotorSimple.Direction.REVERSE);
        slideR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void moveSlides(int targetPosition){
        // Sync target positions
        int currentPositionL = slideL.getCurrentPosition();
        slideL.setTargetPosition(currentPositionL + targetPosition); // Add target offset to current position
        slideR.setTargetPosition(currentPositionL + targetPosition); // Sync second motor to first

        slideL.setMode(RunMode.RUN_TO_POSITION);
        slideR.setMode(RunMode.RUN_TO_POSITION);
        slideL.setVelocity(SLIDE_VELOCITY);
        slideR.setVelocity(SLIDE_VELOCITY);
    }
    public void resetSlides() {
        slideL.setMode(RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        moveSlides(SLIDES_RETRACTED);
    }

    public class slidesExtend implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            moveSlides(SLIDES_EXTENDED);
            telemetryPacket.put("Slide Left Position", slideL.getCurrentPosition());
            telemetryPacket.put("Slide Right Position", slideR.getCurrentPosition());
            return false;
            //return slideL.isBusy() && slideR.isBusy();
        }
    }
    public Action slidesExtend(){
        return new slidesExtend();
    }
    public class slidesRetract implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            moveSlides(SLIDES_RETRACTED);
            telemetryPacket.put("Slide Left Position", slideL.getCurrentPosition());
            telemetryPacket.put("Slide Right Position", slideR.getCurrentPosition());
            return false;
            //return slideL.isBusy() && slideR.isBusy();
        }
    }
    public Action slidesRetract(){
        return new slidesRetract();
    }


}
