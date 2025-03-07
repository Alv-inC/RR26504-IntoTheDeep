package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;


@TeleOp
public class liftPIDTUNING extends OpMode {
    private Lift lift;

    @Override
    public void init() {
        lift = new Lift(hardwareMap);
    }

    @Override
    public void loop() {
//        if(gamepad1.dpad_up){
//            lift.setTargetPosition(100);
//        }
//        else if(gamepad1.dpad_down){
//            lift.setTargetPosition(0);
//        }
        lift.update();
    }

}
