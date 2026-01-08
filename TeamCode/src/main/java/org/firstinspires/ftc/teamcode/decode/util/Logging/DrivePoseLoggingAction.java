package org.firstinspires.ftc.teamcode.decode.util.Logging;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.decode.util.PoseMessage;

public class DrivePoseLoggingAction implements Action {
    String label;
    String message;
    static ElapsedTime autoStartTimer;
    static ElapsedTime lastLogEndTimer;
    Pose2d poseToLog;
    Follower follower;

    public DrivePoseLoggingAction(Follower follower, String label, boolean isStart) {
        this.follower = follower;
        this.label = label;
        if(isStart) {
            autoStartTimer = null;
        }
    }

    public DrivePoseLoggingAction(Follower follower, String label) {
        this.follower = follower;
        this.label = label;
    }

    public DrivePoseLoggingAction(Follower follower, String label, String message, boolean isStart) {
        this.follower = follower;
        this.label = label;
        this.message = message;

        if(isStart) {
            autoStartTimer = null;
        }
    }

    public DrivePoseLoggingAction(Follower follower, String label, String message) {
        this.follower = follower;
        this.label = label;
        this.message = message;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if(autoStartTimer == null) {
            autoStartTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }

        if(lastLogEndTimer == null) {
            lastLogEndTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }

        Log.d("Drive_Logger",
                         "  [" + this.label + "] "
                + "Estimated Pose: " + new PoseMessage(new Pose2d(new Vector2d(follower.getPose().getX(), follower.getPose().getY()), follower.getPose().getHeading()))
                                 + " | Elapsed time: " + String.format("%3.1f", lastLogEndTimer.milliseconds())
                        + (message != null? " | { " + this.message + " }": "")
                        + " | Auto Timer (s): " + String.format("%.3f",autoStartTimer.milliseconds()));

        lastLogEndTimer.reset();
        return false;
    }
}

