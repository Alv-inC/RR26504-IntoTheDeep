package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ChainActions {
    //initialize stuffs
    private Lift lift;

    public ChainActions(HardwareMap hardwareMap){
        lift = new Lift(hardwareMap);
    }

    public Action liftUpScore(double position){
        return new SequentialAction(
                //add the chain of commands here
                lift.setTargetPositionAction(position),
                lift.waitUntilDone()
        );
    }
}
