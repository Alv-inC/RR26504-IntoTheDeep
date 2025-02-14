package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.cameraProcessor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ChainActions {
    //initialize stuffs
    private Lift lift;
    private Turret turret;
    private InitializeTeleOp externTele;

    public ChainActions(HardwareMap hardwareMap){
        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);
        externTele = new InitializeTeleOp();
        externTele.initialize(hardwareMap);
    }

    public Action startPosition(){
        return new SequentialAction(
                //add the chain of commands here
        new InstantAction(() -> externTele.lext.setPosition(0.05)),
                new InstantAction(() -> externTele.rext.setPosition(0.05)),
                new InstantAction(() -> externTele.claw.setPosition(0.6)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.34)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.34)),
                new InstantAction(() -> externTele.primary.setPosition(0.33)),
                new InstantAction(() -> turret.setTargetPosition(0)),
                new InstantAction(() -> lift.setTargetPosition(30))
        );
    }

    //
    public Action scorePosition(){
        return new SequentialAction(
                new InstantAction(() -> externTele.primary.setPosition(0.1)),
                new InstantAction(() -> externTele.rotation.setPosition(0.47)),
                new InstantAction(() -> externTele.lext.setPosition(0)),
                new InstantAction(() -> externTele.rext.setPosition(0)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.37)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.37)),
                new InstantAction(() -> turret.setTargetPosition(-1250)),
                new SleepAction(1.6),
                new InstantAction(() -> externTele.primary.setPosition(0.2)),
                new InstantAction(() -> lift.setTargetPosition(770)),
                new InstantAction(() -> externTele.lext.setPosition(0.12)),
                new InstantAction(() -> externTele.rext.setPosition(0.12))
        );
    }

    public Action scoreSpecimen(){
        return new SequentialAction(
                new InstantAction(() -> externTele.lsecondary.setPosition(0.19)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.19)),
                new InstantAction(() -> externTele.primary.setPosition(0.3)),
                new InstantAction(() -> externTele.rotation.setPosition(0.47)),
                new InstantAction(() -> lift.setTargetPosition(660)),
                new SleepAction(0.8),
                new InstantAction(() -> externTele.claw.setPosition(0.73))
        );
    }

    public Action grabPosition(){
        return new SequentialAction(
                new InstantAction(() -> externTele.lext.setPosition(0.05)),
                new InstantAction(() -> externTele.rext.setPosition(0.05)),
                new InstantAction(() -> externTele.primary.setPosition(0.1)),
                new InstantAction(() -> externTele.rotation.setPosition(0.47)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.37)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.37)),
                new InstantAction(() -> lift.setTargetPosition(30)),
                new InstantAction(() -> turret.setTargetPosition(0)),
                new SleepAction(1.7),
                new InstantAction(() -> externTele.rotation.setPosition(0.47)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.16)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.16)),
                new InstantAction(() -> externTele.primary.setPosition(0.3))

        );
    }

    public Action grabPositionAuto(){
        return new SequentialAction(
                new InstantAction(() -> externTele.primary.setPosition(0.1)),
                new InstantAction(() -> externTele.rotation.setPosition(0.47)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.37)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.37)),
                new InstantAction(() -> lift.setTargetPosition(30)),
                new InstantAction(() -> turret.setTargetPosition(-1250)),
                new SleepAction(1.7),
                new InstantAction(() -> externTele.rotation.setPosition(0.47)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.16)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.16)),
                new InstantAction(() -> externTele.primary.setPosition(0.3))

        );
    }

    public Action scorePositionAuto(){
        return new SequentialAction(
                new InstantAction(() -> externTele.primary.setPosition(0.1)),
                new InstantAction(() -> externTele.rotation.setPosition(0.47)),
                new InstantAction(() -> externTele.lext.setPosition(0)),
                new InstantAction(() -> externTele.rext.setPosition(0)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.37)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.37)),
                new SleepAction(2.3),
                new InstantAction(() -> turret.setTargetPosition(0)),
                new SleepAction(1.6),
                new InstantAction(() -> externTele.primary.setPosition(0.2)),
                new InstantAction(() -> lift.setTargetPosition(800)),
                new InstantAction(() -> externTele.lext.setPosition(0.12)),
                new InstantAction(() -> externTele.rext.setPosition(0.12))
        );
    }
    public Action grabSample(cameraProcessor processor){
        return new SequentialAction(
                new InstantAction(() ->externTele.rotation.setPosition(externTele.rotation.getPosition()+processor.getServoAdjustment())),
                new SleepAction(0.3),
                new InstantAction(() ->externTele.lsecondary.setPosition(0.16)),
                new InstantAction(() ->externTele.rsecondary.setPosition(0.16)),
                new SleepAction(0.25),
                new InstantAction(() ->externTele.claw.setPosition(0.42))
        );
    }

    public Action readyGrab(){
        return new SequentialAction(
                new InstantAction(() -> externTele.claw.setPosition(0.6)),
                new InstantAction(() -> externTele.lext.setPosition(0.22)),
                new InstantAction(() -> externTele.rext.setPosition(0.22)),
                new InstantAction(() -> externTele.lsecondary.setPosition(0.28)),
                new InstantAction(() -> externTele.rsecondary.setPosition(0.28)),
                new InstantAction(() -> externTele.primary.setPosition(0.67)),
                new InstantAction(() -> externTele.rotation.setPosition(0.47))
        );
    }
    public Action scoreLowBasket(){
        return new SequentialAction(
                new SleepAction(1),
                new InstantAction(() -> turret.setTargetPosition(937.5))
        );
    }
    public Action intake(cameraProcessor processor){
        return new SequentialAction(
                new InstantAction(() ->turret.setTargetPosition(turret.getCurrentPosition()+ processor.getTurretAdjustment()*-1)),
            new SleepAction(3),
                new InstantAction(() ->externTele.lext.setPosition(externTele.lext.getPosition()+processor.getExtensionAdjustment())),
                new InstantAction(() ->externTele.rext.setPosition(externTele.lext.getPosition()+processor.getExtensionAdjustment())),
            new SleepAction(0.5),
                new InstantAction(() ->externTele.lext.setPosition(externTele.lext.getPosition()+processor.getExtensionAdjustment())),
                new InstantAction(() ->externTele.rext.setPosition(externTele.lext.getPosition()+processor.getExtensionAdjustment())),
            new SleepAction(0.5),
                new InstantAction(() ->externTele.rotation.setPosition(externTele.rotation.getPosition()+processor.getServoAdjustment())),
            new SleepAction(0.3),
                new InstantAction(() ->externTele.lsecondary.setPosition(0.16)),
                new InstantAction(() ->externTele.rsecondary.setPosition(0.16)),
            new SleepAction(0.3),
                new InstantAction(() ->externTele.claw.setPosition(0.42))
        );
    }
    //prepareGrab
    //Grab
    //neutralPosition
//






}
