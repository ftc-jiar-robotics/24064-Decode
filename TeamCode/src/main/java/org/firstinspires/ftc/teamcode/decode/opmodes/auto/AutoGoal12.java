package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFuturePoseOn;
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

import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.GoalPaths;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.decode.util.Actions;
import org.firstinspires.ftc.teamcode.decode.util.FollowPathAction;
// TODO add priming for shooting
@Configurable
@Autonomous(name = "AutoGoal12", preselectTeleOp = "Main TeleOp")
public class AutoGoal12 extends AbstractAuto{
    private Follower f;
    private GoalPaths path;

    @Override
    protected Pose getStartPose() {
        return GoalPaths.start;
    }
    @Override
    protected void onInit() {

        f = robot.drivetrain;
        path = new GoalPaths(f);

        isFuturePoseOn = true;

        if (Common.isRed != GoalPaths.isPathRed) {
            GoalPaths.isPathRed = !GoalPaths.isPathRed;
            path.mirrorAll();
        }
        Common.robot.shooter.setGoalAlliance();
        path.goal12Build();
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
        path.thirdShoot.getPath(1).setTValueConstraint(0.88);
        path.thirdShoot.getPath(0).setTValueConstraint(0.88);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> Log.d("AutoGoal", "START_SHOOT_THIRD")),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.thirdShoot, .01, 0, f, "speed_up_3"), // speed up to dash to third set of balls
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                    new InstantAction(() -> f.setMaxPower(.5)),
                                                    RobotActions.setIntake(1, 0)
                                        ),
                                path.thirdShoot, 0.8, 0, f, "slow_down_3"), // slow down to intake balls
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.thirdShoot, .01, 2, f, "speed_up_3_post_intake"), // speed up to dash back to close triangle and start shooting procedure
                                new FollowPathAction(f, path.thirdShoot, true)
                        ),

                        RobotActions.shootArtifacts(3, 3),
                        new FollowPathAction(f, path.goalLeave),
                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_THIRD"))
                )
        );

        robot.actionScheduler.runBlocking();
    }

    private void shootSecond() {
        path.secondIntake.getPath(2).setTValueConstraint(0.88);
        path.secondIntake.getPath(1).setTValueConstraint(0.8);
        path.secondIntake.getPath(0).setTValueConstraint(0.88);

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> Log.d("AutoGoal", "START_SHOOT_SECOND")),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.secondIntake, 0.01, 0, f, "speed_up_2"), // speed up to dash to second balls
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(.5)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.secondIntake, 0.8, 0, f, "slow_down_2"), // slow down to intake balls
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.secondIntake, 0.01, 2, f, "speed_up_2_post_intake"), // lets go fast after intake balls, back to triangle to shoot
                                new FollowPathAction(f, path.secondIntake) //dashes to second 3 balls, slows down and starts intake at halfway point in path
                        ),

                        //shoots first 3 balls
                        RobotActions.shootArtifacts(3, 3),

                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_SECOND"))
                )
        );

        robot.actionScheduler.runBlocking();
    }

    private void shootFirst() {
        path.firstIntake.getPath(2).setTValueConstraint(0.88);
        path.firstIntake.getPath(1).setTValueConstraint(0.88);
        path.firstIntake.getPath(0).setTValueConstraint(0.88);

        robot.actionScheduler.addAction(
             new SequentialAction(
                     new InstantAction(() -> Log.d("AutoGoal", "START_SHOOT_FIRST")),
                     new ParallelAction(
                             new Actions.CallbackAction(
                                     new ParallelAction(
                                             new InstantAction(() -> f.setMaxPower(.5)),
                                             RobotActions.setIntake(1, 0)
                                     ),
                                     path.firstIntake, 0.3, 0, f, "slow_down_1"), // slow down to intake balls
                             new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.firstIntake, 0.01, 2, f, "speed_up_1_post_intake"), // speed up after intake
                             new FollowPathAction(f, path.firstIntake, true) // dashes to first 3 balls, starts intake and slows down near halfway points of path, then goes to gate and releases scored balls
                     ),
                     new SleepAction(0.6), //hits the bar to let out scored balls, and sits there for half a second
                     new FollowPathAction(f, path.firstShoot, true), //dashes at max speed back to line to prepare shooting sequence

                     //shoots first 3 balls
                     RobotActions.shootArtifacts(3, 3),

                     new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_FIRST"))
        ));


        robot.actionScheduler.runBlocking();

    }

    private void shootPreload() {
        path.shootPreload.getPath(0).setTValueConstraint(0.88);

        robot.actionScheduler.addAction(
            new SequentialAction( //dashes to line and shoots preloaded 3 balls
                    new InstantAction(() -> Log.d("AutoGoal", "START_SHOOT_PRELOAD")),
                    new FollowPathAction(f, path.shootPreload, true),
                    RobotActions.shootArtifacts(3, 3),
                    new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_PRELOAD"))
            )
        );

        robot.actionScheduler.runBlocking();
    }
}
