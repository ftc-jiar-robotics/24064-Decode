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

import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.AudiencePaths;
import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.GoalPaths;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.decode.util.Actions;
import org.firstinspires.ftc.teamcode.decode.util.FollowPathAction;

@Configurable
@Autonomous(name = "AutoGoal15", preselectTeleOp = "Main TeleOp")

public class AutoGoal15 extends AbstractAuto{
    private GoalPaths path;

    @Override
    protected Pose getStartPose() {
        return GoalPaths.start;
    }
    @Override
    protected void onInit() {

        f = robot.drivetrain;
        path = new GoalPaths(f);

        isFuturePoseOn = false;

        if (Common.isRed != GoalPaths.isPathRed) {
            GoalPaths.isPathRed = !GoalPaths.isPathRed;
            path.mirrorAll();
        }
        Common.robot.shooter.setGoalAlliance();
        path.goal15Build();
    }
    @Override
    protected void onRun() {
        shootPreload();
        shootFirst();
        shootSecond();
        shootThird();
        shootHP();
    }


private void shootHP() { //shoot hp? :whatwasyourauton:
    path.humanPlayerIntake0.getPath(0).setTValueConstraint(0.8);
    path.humanPlayerShoot.getPath(1).setTValueConstraint(0.88);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> Log.d("AutoGoal", "START_SHOOT_HP")),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.humanPlayerIntake0, .01, 0, f, "speed_up_hp"), // speed up to dash to third set of balls
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.humanPlayerIntake0, 0.8, 0, f, "slow_down_hp"), // slow down to intake balls
                                new Actions.TimedAction(new FollowPathAction(f, path.humanPlayerIntake0, true), AudiencePaths.MAX_HP_TIME_MS, "firstHPGoal")
                        ),
                        new SleepAction(0.3),
                        new Actions.TimedAction(new FollowPathAction(f, path.humanPlayerIntake1, true), GoalPaths.MAX_HP_TIME_MS, "secondHPGoal"),
                        new SleepAction(0.3),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.humanPlayerShoot, .01, 1, f, "speed_up_hp_post_intake"), // speed up to dash back to close triangle and start shooting procedure
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel()
                                        ),
                                        path.humanPlayerShoot, 0.01, 1, f, "arm_flywheel_and_turret_hp"
                                ),
                                new FollowPathAction(f, path.humanPlayerShoot, true)
                        ),
                        new Actions.UntilConditionAction(() -> getRuntime() > 29, RobotActions.shootArtifacts(3, 1.5)),
                        new FollowPathAction(f, path.goalLeave),
                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_HP"))
                )
        );

        robot.actionScheduler.runBlocking();
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
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.thirdShoot, 0.8, 0, f, "slow_down_3"), // slow down to intake balls
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.thirdShoot, .01, 2, f, "speed_up_3_post_intake"), // speed up to dash back to close triangle and start shooting procedure
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel()
                                        ),
                                        path.thirdShoot, 0.01, 2, f, "arm_flywheel_and_turret_3"
                                ),
                                new FollowPathAction(f, path.thirdShoot, true)
                        ),

                        RobotActions.shootArtifacts(3, 1.5),
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
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.secondIntake, 0.8, 0, f, "slow_down_2"), // slow down to intake balls
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.secondIntake, 0.01, 2, f, "speed_up_2_post_intake"), // lets go fast after intake balls, back to triangle to shoot
                                new FollowPathAction(f, path.secondIntake)//dashes to second 3 balls, slows down and starts intake at halfway point in path
                        ),
                        new SleepAction(0.6), // sleep to let balls roll out of classifier
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel()
                                        ),
                                        path.secondShoot, 0.01, 0, f, "arm_flywheel_and_turret_2"
                                ),
                                new FollowPathAction(f, path.secondShoot) // goes from gate to shoot second set
                        ),

                        //shoots first 3 balls
                        RobotActions.shootArtifacts(3, 1.5),

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
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.firstIntake, 0.3, 0, f, "slow_down_1"), // slow down to intake balls
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.firstIntake, 0.01, 2, f, "speed_up_1_post_intake"), // speed up after intake
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel()
                                        ),
                                        path.firstIntake, 0.01, 2, f, "arm_flywheel_and_turret_1"
                                ),
                                new FollowPathAction(f, path.firstIntake, true) // dashes to first 3 balls, starts intake and slows down near halfway points of path
                        ),

                        //shoots first 3 balls
                        RobotActions.shootArtifacts(3, 1.5),

                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_FIRST"))
                ));


        robot.actionScheduler.runBlocking();

    }

    private void shootPreload() {
        path.shootPreload.getPath(0).setTValueConstraint(0.88);

        robot.actionScheduler.addAction(
                new SequentialAction( //dashes to line and shoots preloaded 3 balls
                        new InstantAction(() -> Log.d("AutoGoal", "START_SHOOT_PRELOAD")),
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel()
                                        ),
                                        path.shootPreload, 0.01, 0, f, "arm_flywheel_and_turret_0"
                                ),
                                new FollowPathAction(f, path.shootPreload, true)
                        ),
                        RobotActions.shootArtifacts(3, 1.5),
                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_PRELOAD"))
                )
        );

        robot.actionScheduler.runBlocking();
    }
}
