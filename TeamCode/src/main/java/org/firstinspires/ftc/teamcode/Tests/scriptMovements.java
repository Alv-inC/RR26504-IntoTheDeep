package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.InitializeTeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import java.util.ArrayList;
import java.util.List;

import page.j5155.expressway.ftc.actions.ActionOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Script Movements", group = "Testing")
public class scriptMovements extends ActionOpMode {
    private InitializeTeleOp externTele;
    private List<Action> runningActions = new ArrayList<>();
    private Turret turret;
    private Lift lift;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, 1);
        lift = new Lift(hardwareMap, 1);
        externTele = new InitializeTeleOp();
        externTele.initialize(hardwareMap);
        externTele.initialize(hardwareMap);
        externTele.lext.setPosition(0.05);
        externTele.rext.setPosition(0.05);
        externTele.claw.setPosition(0.6);
        turret.setTargetPosition(0);
    }

    @Override
    public void loop() {

        // Update running actions
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;



        if(gamepad1.a){
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> externTele.lext.setPosition(0.1)),
                    new InstantAction(() -> externTele.rext.setPosition(0.1)),
                    new InstantAction(() -> externTele.rotation.setPosition(0.4)),
                    new SleepAction(1),
                    new InstantAction(() -> externTele.rotation.setPosition(0.53)),
                    new InstantAction(() -> externTele.claw.setPosition(0.6)),
                    new SleepAction(1),
                    new InstantAction(() -> externTele.claw.setPosition(0.42)),
                    new InstantAction(() -> lift.setTargetPosition(30))
            ));
        }

        turret.update();
        lift.update();
    }
}
