package org.firstinspires.ftc.teamcode.valueTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.ChainActions;

import java.util.ArrayList;
import java.util.List;

import page.j5155.expressway.ftc.actions.ActionOpMode;

@Config
@TeleOp
public class movementTest extends ActionOpMode {
    public static boolean strafeV, rotate, strafeH, basketPosition, intakePosition = false;
    public static int h, v, rotateTicks = 0;

    MecanumDrive drive;
    private List<Action> runningActions = new ArrayList<>();
    ChainActions chain;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        chain = new ChainActions(hardwareMap);
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

        if(basketPosition){
            basketPosition = false;
            runningActions.add(chain.basketPosition(drive));
        }
        if(intakePosition){
            intakePosition = false;
            runningActions.add(chain.intakePosition());
        }


        telemetry.addData("leftFront encoder: ", drive.leftFront.getCurrentPosition());
        telemetry.addData("rightFront encoder: ", drive.rightFront.getCurrentPosition());
        telemetry.addData("leftBack encoder: ", drive.leftBack.getCurrentPosition());
        telemetry.addData("pose", drive.pose.position);
        telemetry.update();
    }
}
