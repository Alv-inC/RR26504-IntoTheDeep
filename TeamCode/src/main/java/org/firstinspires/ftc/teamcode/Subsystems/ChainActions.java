package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ChainActions {
    //initialize stuffs
    private Lift lift;

    public ChainActions(HardwareMap hardwareMap, Telemetry telemetry){
        lift = new Lift(hardwareMap, telemetry);
    }

    public Action liftUpScore(double position){
        return new SequentialAction(
                //add the chain of commands here
                lift.setTargetPositionAction(position),
                lift.waitUntilDone()
        );
    }







}
