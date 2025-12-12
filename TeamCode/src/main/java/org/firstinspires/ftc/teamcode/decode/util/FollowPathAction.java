package org.firstinspires.ftc.teamcode.decode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class FollowPathAction implements Action {

    private final Follower m_follower;

    private Path m_path;

    private PathChain m_pathChain;
    private boolean holdEnd = false;

    private boolean started = false;

    public FollowPathAction(Follower follower, Path path, boolean holdEnd) {
        this.m_follower = follower;
        this.m_path = path;
        this.holdEnd = holdEnd;
    }

    public FollowPathAction(Follower follower) {
        this.m_follower = follower;
    }

    public FollowPathAction(Follower follower, Path path) {
        this.m_follower = follower;
        this.m_path = path;
    }

    public FollowPathAction(Follower follower, PathChain pathChain, boolean holdEnd) {
        this.m_follower = follower;
        this.m_pathChain = pathChain;
        this.holdEnd = holdEnd;
    }

    public FollowPathAction(Follower follower, PathChain pathChain) {
        this.m_follower = follower;
        this.m_pathChain = pathChain;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if(!started) {
            if(m_path != null) {
                m_follower.followPath(m_path, holdEnd);
            } else if(m_pathChain != null) {
                m_follower.followPath(m_pathChain, holdEnd);
            }
            started = true;
        }

        m_follower.update();
//        Drawing.drawRobot(m_follower);

        return m_follower.isBusy();
    }

    public void breakFollowing() {
        m_follower.breakFollowing();
    }
}
