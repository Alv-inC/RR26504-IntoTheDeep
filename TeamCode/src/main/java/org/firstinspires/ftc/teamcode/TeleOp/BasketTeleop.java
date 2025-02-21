package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ChainActions;
import org.firstinspires.ftc.teamcode.Subsystems.InitializeTeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.cameraProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

import page.j5155.expressway.ftc.actions.ActionOpMode;

@Config
@TeleOp
public class BasketTeleop extends ActionOpMode {
    private InitializeTeleOp externTele;  // Use the class-level reference

    public static boolean strafeV, rotate, strafeH, joystick = false;
    public static int h, v, rotateTicks = 0;

    MecanumDrive drive;
    private List<Action> runningActions = new ArrayList<>();
    ChainActions chain;

    cameraProcessor processor;
    Turret turret;
    Lift lift;



    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        chain = new ChainActions(hardwareMap);
        processor = new cameraProcessor(new Scalar(100, 0, 0,0), new Scalar(255, 5, 230,255), false);
        turret = new Turret(hardwareMap);
        lift = new Lift(hardwareMap);

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        externTele = new InitializeTeleOp();
        externTele.initialize(hardwareMap);
        externTele.lext.setPosition(0.05);
        externTele.rext.setPosition(0.05);
        externTele.claw.setPosition(0.6);
        turret.setTargetPosition(0);
        lift.setTargetPosition(30);
    }

    @Override
    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;
        drive.updatePoseEstimate();

        if(strafeH){
            strafeH = false;
            runningActions.add(chain.strafeHorizontal(drive, h));

        }
        if(strafeV){
            strafeV = false;
            runningActions.add(chain.strafeVertical(drive, v));

        }
        if(rotate){
            rotate = false;
            runningActions.add(chain.rotateBot(drive, rotateTicks));

        }

        //drivetrain code
        if(joystick) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
        }
        else {
            if (gamepad2.dpad_up) runningActions.add(chain.strafeHorizontal(drive, 55));
            else if (gamepad2.dpad_down) runningActions.add(chain.strafeHorizontal(drive, -55));
            else if (gamepad2.dpad_right) runningActions.add(chain.rotateBot(drive, -90));
            else if (gamepad2.dpad_left) runningActions.add(chain.rotateBot(drive, 90));
        }

        drive.updatePoseEstimate();

        if (gamepad1.dpad_left) joystick = true;
        if(gamepad2.dpad_right) joystick = false;

        if(gamepad2.dpad_down){
            runningActions.add(new SequentialAction(
                    new InstantAction(() ->     externTele.claw.setPosition(0.6)),
                    new InstantAction(() -> externTele.lext.setPosition(0.22)),
                    new InstantAction(() -> externTele.rext.setPosition(0.22)),
                    new InstantAction(() -> externTele.lsecondary.setPosition(0.28)),
                    new InstantAction(() -> externTele.rsecondary.setPosition(0.28)),
                    new InstantAction(() -> externTele.primary.setPosition(0.67)),
                    new InstantAction(() -> externTele.rotation.setPosition(0.47))
            ));
        }

        boolean previousButtonState1a = false; // Stores the previous state of the button

        if (gamepad2.a && !previousButtonState1a) {
            runningActions.add(new SequentialAction(
                    new InstantAction(() ->turret.setTargetPosition(turret.getCurrentPosition()+ processor.getTurretAdjustment()*-1)),
                    new SleepAction(1.5),
                    new InstantAction(() ->externTele.lext.setPosition(externTele.lext.getPosition()+processor.getExtensionAdjustment())),
                    new InstantAction(() ->externTele.rext.setPosition(externTele.lext.getPosition()+processor.getExtensionAdjustment())),
                    new SleepAction(0.5),
                    new InstantAction(() ->externTele.rotation.setPosition(externTele.rotation.getPosition()+processor.getServoAdjustment())),
                    new SleepAction(0.3),
                    new InstantAction(() ->externTele.lsecondary.setPosition(0.16)),
                    new InstantAction(() ->externTele.rsecondary.setPosition(0.16)),
                    new SleepAction(0.3),
                    new InstantAction(() ->externTele.claw.setPosition(0.42))
            ));
        }
        previousButtonState1a = gamepad2.a;


        telemetry.addData("leftFront encoder: ", drive.leftFront.getCurrentPosition());
        telemetry.addData("rightFront encoder: ", drive.rightFront.getCurrentPosition());
        telemetry.addData("leftBack encoder: ", drive.leftBack.getCurrentPosition());
        telemetry.addData("pose", drive.pose.position);
        telemetry.update();
    }
}
