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
import org.firstinspires.ftc.teamcode.decode.util.PathBuilder;

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
        robot.limelight.getLimelight().stop();
        robot.limelight.getLimelight().close();
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
    path.humanPlayerIntake1_5.getPath(0).setTValueConstraint(0.775);
    path.humanPlayerShoot.getPath(0).setTValueConstraint(0.88);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> Log.d("AutoGoal", "START_SHOOT_HP")),
                        PathBuilder.buildPath(path.humanPlayerIntake0, f, true, GoalPaths.MAX_HP_GOING_MS,
                                new PathBuilder.PathBuilderCallback(new InstantAction(() -> f.setMaxPower(1)), path.humanPlayerIntake0, .01, 0, f, "speed_up_hp"), // speed up to dash to third set of balls
                                new PathBuilder.PathBuilderCallback(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.humanPlayerIntake0, 0.3, 0, f, "slow_down_hp") // slow down to intake balls
                        ),
                        new SleepAction(0.3),
                        new Actions.TimedAction(new FollowPathAction(f, path.humanPlayerIntake1, true), GoalPaths.MAX_HP_TIME_MS, "firstHPGoal"),
                        new SleepAction(0.3),
                        new Actions.TimedAction(new FollowPathAction(f, path.humanPlayerIntake1_5, false), GoalPaths.MAX_HP_TIME_MS, "secondHPGoal"),
                        new Actions.UntilConditionAction(() -> getRuntime() > GoalPaths.LEAVE_TIME, PathBuilder.buildPath(path.humanPlayerShoot, f, true,
                                new PathBuilder.PathBuilderCallback(new InstantAction(() -> f.setMaxPower(1)), path.humanPlayerShoot, .01, 0, f, "speed_up_hp_post_intake"), // speed up to dash back to close triangle and start shooting procedure
                                new PathBuilder.PathBuilderCallback(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel(),
                                                RobotActions.setIntake(0.25, 0)
                                        ),
                                        path.humanPlayerShoot, 0.01, 0, f, "arm_flywheel_and_turret_hp"
                                )
                        )),
                        new Actions.UntilConditionAction(() -> getRuntime() > GoalPaths.LEAVE_TIME, RobotActions.shootArtifacts(3)),
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
                        PathBuilder.buildPath(path.thirdShoot, f, true,
                                new PathBuilder.PathBuilderCallback(new InstantAction(() -> f.setMaxPower(1)), path.thirdShoot, .01, 0, f, "speed_up_3"), // speed up to dash to third set of balls
                                new PathBuilder.PathBuilderCallback(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.thirdShoot, 0.3, 0, f, "slow_down_3"), // slow down to intake balls
                                new PathBuilder.PathBuilderCallback(new InstantAction(() -> f.setMaxPower(1)), path.thirdShoot, .01, 2, f, "speed_up_3_post_intake"), // speed up to dash back to close triangle and start shooting procedure
                                new PathBuilder.PathBuilderCallback(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel(),
                                                RobotActions.setIntake(0.25, 0)
                                        ),
                                        path.thirdShoot, 0.01, 2, f, "arm_flywheel_and_turret_3"
                                )
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
                        PathBuilder.buildPath(path.secondIntake, f, false,
                                new PathBuilder.PathBuilderCallback(new InstantAction(() -> f.setMaxPower(1)), path.secondIntake, 0.01, 0, f, "speed_up_2"), // speed up to dash to second balls
                                new PathBuilder.PathBuilderCallback(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.secondIntake, 0.3, 0, f, "slow_down_2"), // slow down to intake balls
                                new PathBuilder.PathBuilderCallback(new InstantAction(() -> f.setMaxPower(1)), path.secondIntake, 0.01, 2, f, "speed_up_2_post_intake") // lets go fast after intake balls, back to triangle to shoot
                        ),
                        new SleepAction(0.6), // sleep to let balls roll out of classifier
                        PathBuilder.buildPath(path.secondShoot, f, false,
                                new PathBuilder.PathBuilderCallback(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel(),
                                                RobotActions.setIntake(0.25, 0)
                                        ),
                                        path.secondShoot, 0.01, 0, f, "arm_flywheel_and_turret_2"
                                )
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
                        PathBuilder.buildPath(path.firstIntake, f, true,
                                new PathBuilder.PathBuilderCallback(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.firstIntake, 0.3, 0, f, "slow_down_1"), // slow down to intake balls
                                new PathBuilder.PathBuilderCallback(new InstantAction(() -> f.setMaxPower(1)), path.firstIntake, 0.01, 2, f, "speed_up_1_post_intake"), // speed up after intake
                                new PathBuilder.PathBuilderCallback(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel(),
                                                RobotActions.setIntake(0.25, 0)
                                        ),
                                        path.firstIntake, 0.01, 2, f, "arm_flywheel_and_turret_1"
                                )
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
                        PathBuilder.buildPath(path.shootPreload, f, true,
                                new PathBuilder.PathBuilderCallback(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel()
                                        ),
                                        path.shootPreload, 0.01, 0, f, "arm_flywheel_and_turret_0"
                                )
                        ),
                        RobotActions.shootArtifacts(3, 4),
                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_PRELOAD"))
                )
        );

        robot.actionScheduler.runBlocking();
    }
}
