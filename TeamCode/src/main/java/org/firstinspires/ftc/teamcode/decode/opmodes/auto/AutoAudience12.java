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
@Autonomous(name = "AutoAudience12", preselectTeleOp = "Main TeleOp")
public class AutoAudience12 extends AbstractAuto{
    private AudiencePaths path;

    @Override
    protected Pose getStartPose() {
        return AudiencePaths.start;
    }
    @Override
    protected void onInit() {

        f = robot.drivetrain;
        path = new AudiencePaths(f);

        isFuturePoseOn = false;

        if (Common.isRed != AudiencePaths.isPathRed) {
            AudiencePaths.isPathRed = !AudiencePaths.isPathRed;
            path.mirrorAll();
        }
        Common.robot.shooter.setGoalAlliance();
        path.audience12Build();
    }
    @Override
    protected void onRun() {
        shootPreload();
        shootFirst();
        shootHPFirst();
        shootHPSecond();
        shootHPThird();
    }


    private void shootHPThird() {
        path.humanPlayerIntake2.getPath(0).setTValueConstraint(0.8);
        path.humanPlayerIntake3.getPath(0).setTValueConstraint(0.775);
        path.humanPlayerShoot2.getPath(1).setTValueConstraint(0.88);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> Log.d("AutoAudience", "START_SHOOT_HP_THIRD")),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.humanPlayerIntake2, .01, 0, f, "speed_up_hp"), // speed up to dash to third set of balls
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.humanPlayerIntake2, 0.8, 0, f, "slow_down_hp_3"), // slow down to intake balls
                                new Actions.TimedAction(new FollowPathAction(f, path.humanPlayerIntake2, false), AudiencePaths.MAX_HP_TIME_MS, "firstHPAudience")
                        ),
                        new SleepAction(0.3),
                        new Actions.TimedAction(new FollowPathAction(f, path.humanPlayerIntake3, false), AudiencePaths.MAX_HP_TIME_MS, "secondHPAudience"),
                        new SleepAction(0.3),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.humanPlayerShoot2, .01, 1, f, "speed_up_hp_post_intake"), // speed up to dash back to close triangle and start shooting procedure
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel()
                                        ),
                                        path.humanPlayerShoot2, 0.01, 1, f, "arm_flywheel_and_turret_hp_3"
                                ),
                                new FollowPathAction(f, path.humanPlayerShoot2, false)
                        ),

                        new Actions.UntilConditionAction(() -> getRuntime() > 29, RobotActions.shootArtifacts(3, 2.5)),
                        new FollowPathAction(f, path.goalLeave),
                        new InstantAction(() -> Log.d("AutoAudience", "END_SHOOT_HP_THIRD"))
                )
        );

        robot.actionScheduler.runBlocking();
    }

    private void shootHPSecond() {
        path.humanPlayerIntake2.getPath(0).setTValueConstraint(0.8);
        path.humanPlayerIntake3.getPath(0).setTValueConstraint(0.775);
        path.humanPlayerShoot2.getPath(1).setTValueConstraint(0.88);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> Log.d("AutoAudience", "START_SHOOT_HP_SECOND")),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.humanPlayerIntake2, .01, 0, f, "speed_up_hp"), // speed up to dash to third set of balls
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.humanPlayerIntake2, 0.8, 0, f, "slow_down_hp_2"), // slow down to intake balls
                                new Actions.TimedAction(new FollowPathAction(f, path.humanPlayerIntake2, false), AudiencePaths.MAX_HP_TIME_MS, "thirdHPAudience")
                        ),
                        new InstantAction(() -> Log.d("thirdHPAudienceDEBUG", "Timed action has FINISHED")),
                        new SleepAction(0.3),
                        new Actions.TimedAction(new FollowPathAction(f, path.humanPlayerIntake3, false), AudiencePaths.MAX_HP_TIME_MS, "fourthHPAudience"),
                        new InstantAction(() -> Log.d("fourthHPAudienceDEBUG", "Timed action has FINISHED")),
                        new SleepAction(0.3),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.humanPlayerShoot2, .01, 1, f, "speed_up_hp_post_intake"), // speed up to dash back to close triangle and start shooting procedure
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel()
                                        ),
                                        path.humanPlayerShoot2, 0.01, 1, f, "arm_flywheel_and_turret_hp_2"
                                ),
                                new FollowPathAction(f, path.humanPlayerShoot2, false)
                        ),

                        RobotActions.shootArtifacts(3, 2.5),
                        new InstantAction(() -> Log.d("AutoAudience", "END_SHOOT_SECOND"))
                )
        );

        robot.actionScheduler.runBlocking();
    }

    private void shootHPFirst() {
        path.humanPlayerIntake0.getPath(0).setTValueConstraint(0.8);
        path.humanPlayerIntake1.getPath(0).setTValueConstraint(0.775);
        path.humanPlayerShoot1.getPath(1).setTValueConstraint(0.88);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> Log.d("AutoAudience", "START_SHOOT_HP_FIRST")),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.humanPlayerIntake0, .01, 0, f, "speed_up_hp"), // speed up to dash to third set of balls
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.humanPlayerIntake0, 0.8, 0, f, "slow_down_hp_1"), // slow down to intake balls

                                new Actions.TimedAction(new FollowPathAction(f, path.humanPlayerIntake0, false), AudiencePaths.MAX_HP_TIME_MS, "firstHpAudience")
                        ),
                        new InstantAction(() -> Log.d("firstHPAudienceDEBUG", "Timed action has FINISHED")),
                        new SleepAction(0.3),
                        new Actions.TimedAction(new FollowPathAction(f, path.humanPlayerIntake1, false), AudiencePaths.MAX_HP_TIME_MS, "secondHPAudience"),
                        new InstantAction(() -> Log.d("secondHPAudienceDEBUG", "Timed action has FINISHED")),
                        new SleepAction(0.3),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.humanPlayerShoot1, .01, 1, f, "speed_up_hp_post_intake"), // speed up to dash back to close triangle and start shooting procedure
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel()
                                        ),
                                        path.humanPlayerShoot1, 0.01, 1, f, "arm_flywheel_and_turret_hp_1"
                                ),
                                new FollowPathAction(f, path.humanPlayerShoot1, false)
                        ),

                        RobotActions.shootArtifacts(3, 2.5),
                        new InstantAction(() -> Log.d("AutoAudience", "END_SHOOT_HP_FIRST"))
                )
        );

        robot.actionScheduler.runBlocking();
    }

    private void shootFirst() {
        path.firstShoot.getPath(2).setTValueConstraint(0.88);
        path.firstShoot.getPath(1).setTValueConstraint(0.88);
        path.firstShoot.getPath(0).setTValueConstraint(0.88);

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> Log.d("AutoAudience", "START_SHOOT_FIRST")),
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.firstShoot, 0.3, 0, f, "slow_down_first"), // slow down to intake balls
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.firstShoot, 0.01, 2, f, "speed_up_1_post_intake"), // speed up after intake
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel()
                                        ),
                                        path.firstShoot, 0.01, 2, f, "arm_flywheel_and_turret_1"
                                ),
                                new FollowPathAction(f, path.firstShoot, false) // dashes to first 3 balls, starts intake and slows down near halfway points of path
                        ),

                        //shoots first 3 balls
                        RobotActions.shootArtifacts(3, 2.5),

                        new InstantAction(() -> Log.d("AutoAudience", "END_SHOOT_FIRST"))
                ));


        robot.actionScheduler.runBlocking();

    }

    private void shootPreload() {
        robot.actionScheduler.addAction(
                new SequentialAction( //dashes to line and shoots preloaded 3 balls
                        new FollowPathAction(f, path.preload),
                        RobotActions.shootArtifacts(3, 4),
                        new InstantAction(() -> Log.d("AutoAudience", "END_SHOOT_PRELOAD"))
                )
        );
        robot.actionScheduler.runBlocking();
    }
}
