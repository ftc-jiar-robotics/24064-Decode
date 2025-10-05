package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.CustomPaths;
import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.Paths;
import org.firstinspires.ftc.teamcode.decode.util.Actions;
import org.firstinspires.ftc.teamcode.decode.util.FollowPathAction;

public class AutoClose extends AbstractAuto{
    private Follower f;
    private Paths path;

    @Override
    protected Pose getStartPose() {
        return null;
    }
    @Override
    protected void onInit() {
        f = robot.drivetrain;
        path = new Paths(f);
        path.buildClose();
    }
    @Override
    protected void onRun() {
        runPath();
    }

    private void runPath() {
        robot.actionScheduler.addAction(
            new FollowPathAction(f, path.close)
        );

        robot.actionScheduler.runBlocking();
    }
}
