package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class IDK {
    public static class RaceParallelCommand implements Action {
        private final Action[] actions;
        public RaceParallelCommand(Action... actions){
            this.actions = actions;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            boolean finished = true;
            for(Action action : actions) finished = finished && action.run(telemetryPacket);
            return finished;

        }

    }
}

