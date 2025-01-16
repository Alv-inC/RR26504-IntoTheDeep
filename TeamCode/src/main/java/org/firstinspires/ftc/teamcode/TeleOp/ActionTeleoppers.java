package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.InitializeTeleOp;
import java.util.ArrayList;
import java.util.List;

import page.j5155.expressway.ftc.actions.ActionOpMode;

@TeleOp(name = "pray")
public class ActionTeleoppers extends ActionOpMode {

    private InitializeTeleOp externTele;  // Use the class-level reference
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize externTele at the class level
        externTele = new InitializeTeleOp();
        externTele.initialize(hardwareMap, telemetry);  // Initialize using hardwareMap and telemetry

        // Set up vision processor (if necessary)
        // final OpenCVTEST.CameraStreamProcessor processor = new OpenCVTEST.CameraStreamProcessor();
        // VisionPortal visionPortal = new VisionPortal.Builder()
        //        .addProcessor(processor)
        //        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        //        .build();
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

        // Trigger actions when gamepad1.x is pressed
        if (gamepad1.x) {
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> externTele.lext.setPosition(0.5)),
                    new InstantAction(() -> externTele.rext.setPosition(0.15)),
                    new InstantAction(() -> externTele.lsecondary.setPosition(0.22)),
                    new InstantAction(() -> externTele.rsecondary.setPosition(0.22)),
                    new InstantAction(() -> externTele.primary.setPosition(0.7))
            ));
        }
        if (gamepad1.a) {
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> externTele.lext.setPosition(0.5)),
                    new InstantAction(() -> externTele.rext.setPosition(0.15)),
                    new InstantAction(() -> externTele.lsecondary.setPosition(0.22)),
                    new InstantAction(() -> externTele.rsecondary.setPosition(0.22)),
                    new InstantAction(() -> externTele.primary.setPosition(0.7))
            ));
        }


        //retract to base position
        if(gamepad1.y){
            runningActions.add(new SequentialAction(
            new InstantAction(() -> externTele.rotation.setPosition(0.75)),
            //turret.setTargetPosition(turretPosition);
            new InstantAction(() -> externTele.lsecondary.setPosition(0.34)),
            new InstantAction(() -> externTele.rsecondary.setPosition(0.34)),
            new InstantAction(() -> externTele.primary.setPosition(0.33)),
            new InstantAction(() -> externTele.lext.setPosition(0)),
            new InstantAction(() -> externTele.rext.setPosition(0))
            ));
        }
        //transfer
        if(gamepad1.dpad_left){
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> externTele.claw.setPosition(0.72)),
            new InstantAction(() -> externTele.lext.setPosition(0.07)),
            new InstantAction(() -> externTele.lext.setPosition(0.07))
            ));
        }
        ////score specimen
                    if(gamepad1.dpad_down){
                        runningActions.add(new SequentialAction(
                        new InstantAction(() -> externTele.lsecondary.setPosition(0.17)),
                        new InstantAction(() -> externTele.rsecondary.setPosition(0.17)),
                        //wait
                        new InstantAction(() -> externTele.primary.setPosition(0.31)),
                        new InstantAction(() -> externTele.rotation.setPosition(0.47))
                        //slide raise
                        //driver move bot forward to attach
                        ));
                    }
        //            //get specimen
                    if(gamepad1.dpad_right){
                        runningActions.add(new SequentialAction(
                        new InstantAction(() -> externTele.lsecondary.setPosition(0.22)),
                        new InstantAction(() -> externTele.rsecondary.setPosition(0.22)),
                        //wait
                        new InstantAction(() -> externTele.primary.setPosition(0.6)),
                        new InstantAction(() -> externTele.rotation.setPosition(0.47))
                        //driver close claw
                        ));
                    }
                    //claw open close
                    if(gamepad1.right_bumper) externTele.claw.setPosition(0.15);
                    if(gamepad1.left_bumper)  externTele.claw.setPosition (0.72);



        //
        //if(gamepad1.a){
        //                //turret.setTargetPosition(turret.getCurrentPosition()+ processor.calculateTurretAdjustment);
        //                waitWithoutStoppingRobot(300);
        //                lext.setPosition(lext.getPosition()+processor.getExtensionAdjustment());
        //                rext.setPosition(rext.getPosition()+processor.getExtensionAdjustment());
        //                waitWithoutStoppingRobot(300);
        //                rotation.setPosition(rotation.getPosition()+processor.getServoAdjustment());
        //                waitWithoutStoppingRobot(300);
        //                lsecondary.setPosition(0.17);
        //                rsecondary.setPosition(0.17);
        //                waitWithoutStoppingRobot(300);
        //                claw.setPosition(0.5);
        //            }
    }
}
