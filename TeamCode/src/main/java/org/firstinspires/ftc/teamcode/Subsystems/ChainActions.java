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
    private Turret turret;

    public ChainActions(HardwareMap hardwareMap){
        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);
    }

    public Action liftUpScore(double position){
        return new SequentialAction(
                //add the chain of commands here
                lift.setTargetPositionAction(position),
                lift.waitUntilDone()
        );
    }







}
