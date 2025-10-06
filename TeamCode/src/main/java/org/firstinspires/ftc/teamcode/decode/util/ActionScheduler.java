package org.firstinspires.ftc.teamcode.decode.util;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;

import java.util.LinkedList;
import java.util.Queue;

// Inspired by QC 21229
// An alternative to Actions.runBlocking() that doesn't block. Main use-case is for tele-op.
// Call run() in every loop and add actions as needed.
public final class ActionScheduler {
    final Queue<Action> actions = new LinkedList<>();
    final FtcDashboard dash = FtcDashboard.getInstance();
    final Canvas canvas = new Canvas();

    private Runnable update = null;

    public void setUpdate(Runnable update) {
        this.update = update;
    }

    public void addAction(Action action) {
        if (action != null) {
            actions.add(lockSubsytems());
            actions.add(action);
            actions.add(unlockSubsytems());
        }
    }

    // Won't generate previews
    public void run() {
        if (actions.peek() != null) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

            boolean running = actions.peek().run(packet);
            dash.sendTelemetryPacket(packet);

            if (!running) {
                actions.remove();
            }
        } else if (!actions.isEmpty()) actions.remove();
    }

    public void runBlocking() {
        Action currentAction = actions.peek();
        while (currentAction != null && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();

            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

            boolean running = currentAction.run(packet);
            dash.sendTelemetryPacket(packet);

            update.run();

            if (!running && actions.peek() != null) {
                actions.remove();
            }

            currentAction = actions.peek();
        }
    }

    private Action lockSubsytems() {
        return new InstantAction(() -> {
            robot.intake.setLocked(true);
            robot.shooter.setLocked(true);
        });
    }

    private Action unlockSubsytems() {
        return new InstantAction(() -> {
            robot.intake.setLocked(false);
            robot.shooter.setLocked(false);
        });
    }
}