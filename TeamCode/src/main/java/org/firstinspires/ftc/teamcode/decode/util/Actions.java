package org.firstinspires.ftc.teamcode.decode.util;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.ParametricCallback;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public static class UntilConditionAction implements Action {
        private final Callable<Boolean> check;
        private final Action action;

        public UntilConditionAction(Callable<Boolean> check, Action action) {
            this.check = check;
            this.action = action;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            try {
                if (!check.call()) {
                    return action.run(packet);
                }
                if (action instanceof FollowPathAction) ((FollowPathAction) action).breakFollowing();
                return false;
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }

    public static class TimedAction implements Action {
        private final UntilConditionAction untilConditionAction;
        private long startTime = Long.MAX_VALUE;
        private final String name;
        private final long maxTimeMs;
        private boolean isFirst = true;

        public TimedAction(Action action, long maxTimeMs, String name) {
            this.name = name;
            this.maxTimeMs = maxTimeMs;
            untilConditionAction = new UntilConditionAction(this::isDone, action);
        }

        private boolean isDone() {
            return System.currentTimeMillis() - startTime > maxTimeMs;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (isFirst) {
                isFirst = false;
                startTime = System.currentTimeMillis();
            }

            try {
                if (!untilConditionAction.run(packet)) {
                    startTime = 0;
                    isFirst = false;
                    return false;
                } else return true;
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
            if (!isCalled) return true;
            Log.d("DEBUG_CALLBACK",s+"started");

//            new DrivePoseLoggingAction(f, s, true).run(telemetryPacket);
            return action.run(telemetryPacket);
        }
    }
}
