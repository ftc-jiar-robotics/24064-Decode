package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.Paths;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.decode.util.Actions;
import org.firstinspires.ftc.teamcode.decode.util.FollowPathAction;

@Configurable
@Autonomous(name = "AutoClose18")
public class AutoClose18 extends AbstractAuto {
    private Follower f;
    private Paths path;
    public static int CYCLES = 2;

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
        Common.robot.shooter.setGoalAlliance();
        path.close18Build();
    }
    @Override
    protected void onRun() {
        shootPreload();
        shootSecond();
        for (int i = 0; i < CYCLES; i++) {
            gateCycle();
        }
    }

    private void gateCycle() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new FollowPathAction(f, path.cycleGate)
                )
        );
        robot.actionScheduler.runBlocking();
    }

    private void shootSecond() {
        path.secondIntake.getPath(1).setTValueConstraint(0.8);
        path.secondShoot.getPath(0).setTValueConstraint(0.88);

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> f.setMaxPower(.3)),
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
                        )
                )
        );

        //shoots first 3 balls
        robot.actionScheduler.addAction(RobotActions.shootArtifacts(1));
        robot.actionScheduler.addAction(RobotActions.shootArtifacts(1));
        robot.actionScheduler.addAction(RobotActions.shootArtifacts(1));

        robot.actionScheduler.runBlocking();
    }

    private void shootPreload() {
        path.shootPreload.getPath(0).setTValueConstraint(0.88);

        robot.actionScheduler.addAction(
                new SequentialAction( //dashes to line and shoots preloaded 3 balls
                        new InstantAction(() -> Log.d("AutoClose", "START_SHOOT_PRELOAD")),
                        new FollowPathAction(f, path.shootPreload, true),
                        new InstantAction(() -> Log.d("AutoClose", "END_SHOOT_PRELOAD"))
                )
        );

        //shoots first 3 balls
        robot.actionScheduler.addAction(RobotActions.shootArtifacts(1));
        robot.actionScheduler.addAction(RobotActions.shootArtifacts(1));
        robot.actionScheduler.addAction(RobotActions.shootArtifacts(1));

        robot.actionScheduler.runBlocking();
    }
}
