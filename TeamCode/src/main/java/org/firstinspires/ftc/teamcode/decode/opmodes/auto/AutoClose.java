package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.Paths;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.decode.util.Actions;
import org.firstinspires.ftc.teamcode.decode.util.FollowPathAction;

@Configurable
@Autonomous(name = "AutoClose")
public class AutoClose extends AbstractAuto{
    private Follower f;
    private Paths path;

    @Override
    protected Pose getStartPose() {
        return Paths.start;
    }
    @Override
    protected void onInit() {

        f = robot.drivetrain;
        path = new Paths(f);

        if (Common.isRed != Paths.isPathRed) {
            Paths.isPathRed = !Paths.isPathRed;
            path.mirrorAll();
        }
        Common.robot.shooter.setGoalAlliance(Common.isRed);
        path.buildClose();
    }
    @Override
    protected void onRun() {
        shootPreload();
        shootFirst();
        shootSecond();
        shootThird();
    }

    private void shootThird() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> f.setMaxPower(.3)),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.close, .1, 7, f, "speed_up_4"),
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(.3)), path.close, .8, 7, f, "slow_down_3"),
                                new Actions.CallbackAction(RobotActions.setIntake(1, 0), path.close, 0.8, 7, f, "start_intake_3"),
                                new FollowPathAction(f, path.close.getPath(7)) // slows down to 30% so it turns slowly & shoots second 3 balls, then speeds up, then starts intake at 80% and slows back down to 30% to intake again
                        ),
                        new FollowPathAction(f, path.close.getPath(8)), // intake balls

                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.close, .01, 9, f, "speed_up_5"),
                                new Actions.CallbackAction(RobotActions.setIntake(0,0), path.close, 0.01, 9, f, "end_intake_3"),
                                new FollowPathAction(f, path.close.getPath(9)) // speed up and end intake at 1% of path
                        ),
                        RobotActions.shootArtifacts(3) // shoots third 3 balls
                )
        );

        robot.actionScheduler.runBlocking();
    }

    private void shootSecond() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> f.setMaxPower(.3)),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.close, .1, 4, f, "speed_up_2"),
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(.3)), path.close, .8, 4, f, "slow_down_2"),
                                new Actions.CallbackAction(RobotActions.setIntake(1, 0), path.close, 0.5, 4, f, "start_intake_2"),
                                new FollowPathAction(f, path.close.getPath(4)) // slows down to 30% so it turns slowly & shoots first 3 balls, then speeds up, then starts intake at 50% and slows back down to 30% to intake again
                        ),
                        new ParallelAction(
                                new Actions.CallbackAction(RobotActions.setIntake(0,0), path.close, 0.98, 5, f, "end_intake_2"),
                                new FollowPathAction(f, path.close.getPath(5)) // intake balls and the end intake at 98% of path so hatch opens
                        ),
                        new SleepAction(0.5),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.close, .1, 6, f, "speed_up_3"),
                                new FollowPathAction(f, path.close.getPath(6)) // speed up and go back to line
                                ),
                        RobotActions.shootArtifacts(3) // shoots second 3 balls
                )
        );

        robot.actionScheduler.runBlocking();
    }

    private void shootFirst() {
        PathChain firstPath = f.pathBuilder().addPath(new BezierLine(Paths.end0, Paths.end1)).setTangentHeadingInterpolation().build();
        robot.actionScheduler.addAction(
             new SequentialAction(
                     new InstantAction(() -> Log.d("AutoClose", "START_SHOOT_FIRST")),
                     new ParallelAction(
//                             new Actions.CallbackAction(RobotActions.setIntake(1, 0), path.close, 0.5, 1, f, "start_intake_1"),
                             new Actions.CallbackAction(new InstantAction(() -> Log.d("DEBUG_AUTON","lehoofy")), firstPath, 0.5, 0, f, "start_intake_1"),
//                             new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(.3)), path.close, .8, 1, f, "slow_down_1"),
                             new FollowPathAction(f, firstPath) // starts intake at 80% and start intake at 50% in path
                     ),
                     new InstantAction(() -> Log.d("AutoClose", "MID1_SHOOT_FIRST")),

                     new FollowPathAction(f, path.close.getPath(2)), // intake balls)
                     new InstantAction(() -> Log.d("AutoClose", "MID2_SHOOT_FIRST")),
                     new ParallelAction(
                             new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.close, .01, 3, f, "speed_up_1"),
                             new Actions.CallbackAction(RobotActions.setIntake(0,0), path.close, 0.01, 3, f, "end_intake_1"),
                             new FollowPathAction(f, path.close.getPath(3)) // speed up and end intake at 1% of path
                    ),
                     new InstantAction(() -> Log.d("AutoClose", "END_SHOOT_FIRST"))
        ));
        robot.actionScheduler.addAction(RobotActions.shootArtifacts(3));

        robot.actionScheduler.runBlocking();
    }

    private void shootPreload() {
        path.close.getPath(0).setTValueConstraint(0.88);

        robot.actionScheduler.addAction(
            new SequentialAction(
                    new InstantAction(() -> Log.d("AutoClose", "START_SHOOT_PRELOAD")),
                    new FollowPathAction(f, path.close.getPath(0)),
                    new InstantAction(() -> Log.d("AutoClose", "MID_SHOOT_PRELOAD")),
                    RobotActions.shootArtifacts(1), // shoots preloaded 3 balls
                    RobotActions.shootArtifacts(1), // shoots preloaded 3 balls
                    RobotActions.shootArtifacts(1), // shoots preloaded 3 balls
                    new InstantAction(() -> Log.d("AutoClose", "END_SHOOT_PRELOAD"))
            )
        );

        robot.actionScheduler.runBlocking();
    }
}
