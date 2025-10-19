package org.firstinspires.ftc.teamcode.decode.util;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.ParametricCallback;

import java.util.concurrent.Callable;

public final class Actions {
    public static class RunnableAction implements Action {
        private final Callable<Boolean> action;

        public RunnableAction(Callable<Boolean> action) {
            this.action = action;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            try {
                return action.call();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }

    public static class SingleCheckAction implements Action {
        private final Callable<Boolean> check;
        private final Action action;
        private boolean expired = false;

        public SingleCheckAction(Callable<Boolean> check, Action action) {
            this.check = check;
            this.action = action;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            try {
                if (expired || check.call()) {
                    expired = true;
                    return action.run(packet);
                }
                return false;
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }

    public static class CallbackAction implements Action {
        private final Action action;

        private boolean isCalled = false;
        private final Follower f;
        private final String s;

        public CallbackAction(Action action, PathChain pathChain, double startCondition, int index, Follower f, String s) {
            this.f = f;
            this.action = action;
            this.s = s;
            pathChain.setCallbacks(new ParametricCallback(index, startCondition, f, () -> {
                Log.d("DEBUG_CALLBACK","lehoofy was called " + s);
                isCalled = true;
            }));
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Log.d("DEBUG_CALLBACK",s+"not started");
            if (!isCalled) return true;
            Log.d("DEBUG_CALLBACK",s+"started");

//            new DrivePoseLoggingAction(f, s, true).run(telemetryPacket);
            return action.run(telemetryPacket);
        }
    }
}
