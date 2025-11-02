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
        path.thirdShoot.getPath(2).setTValueConstraint(0.88);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> f.setMaxPower(.3)),
                        RobotActions.armFlywheel(),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.thirdShoot, .01, 0, f, "speed_up_4"), // speed up to dash to third set of balls
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(.5)), path.thirdShoot, 0.8, 0, f, "slow_down_3"), // slow down to intake balls
                                new Actions.CallbackAction(RobotActions.setIntake(1, 0), path.thirdShoot, 0.8, 0, f, "start_intake_3"), // start intake to intake balls
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.thirdShoot, .01, 2, f, "speed_up_5"), // speed up to dash back to close triangle and start shooting procedure
//                                new Actions.CallbackAction(RobotActions.setIntake(0,0), path.thirdShoot, 0.01, 2, f, "end_intake_3"),
                                new FollowPathAction(f, path.thirdShoot, true)
                        ),
                        RobotActions.shootArtifacts(3) // shoots third 3 balls
                )
        );

        robot.actionScheduler.runBlocking();
    }

    private void shootSecond() {
        path.secondIntake.getPath(1).setTValueConstraint(0.88);
        path.secondShoot.getPath(0).setTValueConstraint(0.88);

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> f.setMaxPower(.3)),
                        RobotActions.armFlywheel(),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.secondIntake, .01, 0, f, "speed_up_2"), // speed up to dash to second balls
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(.5)), path.secondIntake, .4, 0, f, "slow_down_2"), // slow down to start in-taking balls
                                new Actions.CallbackAction(RobotActions.setIntake(1, 0), path.secondIntake, 0.5, 0, f, "start_intake_2"), // intake balls
//                                new Actions.CallbackAction(RobotActions.setIntake(0,0), path.secondIntake, 0.98, 1, f, "end_intake_2"),
                                new FollowPathAction(f, path.secondIntake) //dashes to second 3 balls, slows down and starts intake at halfway point in path, then before dashing back to close triangle, hits bar to let all scored balls out.
                        ),
                        new SleepAction(0.5), //hits the bar to let out scored balls, and sits there for half a second
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.secondShoot, .01, 0, f, "speed_up_3"), // speed up to dash back to close triangle
                                new FollowPathAction(f, path.secondShoot, true) //dashes at max speed back to line to prepare shooting sequence
                                ),
                        RobotActions.shootArtifacts(3) // shoots second 3 balls
                )
        );

        robot.actionScheduler.runBlocking();
    }

    private void shootFirst() {
        path.firstShoot.getPath(2).setTValueConstraint(0.88);
        path.firstShoot.getPath(1).setTValueConstraint(0.85);
        robot.actionScheduler.addAction(
             new SequentialAction(
                     new InstantAction(() -> Log.d("AutoClose", "START_SHOOT_FIRST")),
                     RobotActions.armFlywheel(),
                     new ParallelAction(
                             new Actions.CallbackAction(RobotActions.setIntake(1, 0), path.firstShoot, 0.5, 0, f, "start_intake_1"), // start intake of first balls
                             new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(.5)), path.firstShoot, .4, 0, f, "slow_down_1"), // slow down to intake
                             new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.firstShoot, .01, 2, f, "speed_up_1"), // speed up to dash back to close triangle
//                             new Actions.CallbackAction(RobotActions.setIntake(0,0), path.firstShoot, 0.01, 2, f, "end_intake_1"),
                             new FollowPathAction(f, path.firstShoot, true) // dashes to first 3 balls, starts intake and slows down near halfway points of path, then max power dash back to line to prepare shooting sequence
                     ),
                     new InstantAction(() -> Log.d("AutoClose", "END_SHOOT_FIRST"))
        ));
        robot.actionScheduler.addAction(RobotActions.shootArtifacts(1)); //shoots first 3 balls
        robot.actionScheduler.addAction(RobotActions.shootArtifacts(1)); //shoots first 3 balls
        robot.actionScheduler.addAction(RobotActions.shootArtifacts(1)); //shoots first 3 balls

        robot.actionScheduler.runBlocking();
    }

    private void shootPreload() {
        path.shootPreload.getPath(0).setTValueConstraint(0.88);

        robot.actionScheduler.addAction(
            new SequentialAction( //dashes to line and shoots preloaded 3 balls
                    RobotActions.armFlywheel(),
                    new InstantAction(() -> Log.d("AutoClose", "START_SHOOT_PRELOAD")),
                    new FollowPathAction(f, path.shootPreload, true),
                    new InstantAction(() -> Log.d("AutoClose", "MID_SHOOT_PRELOAD")),
                    new InstantAction(() -> Log.d("AutoClose", "END_SHOOT_PRELOAD"))
            )
        );

        robot.actionScheduler.addAction(RobotActions.shootArtifacts(3));
        robot.actionScheduler.runBlocking();
    }
}
