package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;


@TeleOp
public class turretPIDTUNING extends OpMode {
    private Turret turret;

    @Override
    public void init() {
        turret = new Turret(hardwareMap,1);
    }

    @Override
    public void loop() {
//        if(gamepad1.dpad_up){
//            lift.setTargetPosition(100);
//        }
//        else if(gamepad1.dpad_down){
//            lift.setTargetPosition(0);
//        }
        turret.update();
    }

}
