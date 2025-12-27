package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.SLOW_MODE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFuturePoseOn;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.AudiencePaths;
import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.GoalPaths;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.decode.util.Actions;
import org.firstinspires.ftc.teamcode.decode.util.FollowPathAction;

@Configurable
@Autonomous(name = "AutoGoal18", preselectTeleOp = "Main TeleOp")

public class AutoGoal18 extends AbstractAuto{
    private GoalPaths path;
    double slowMode;

    @Override
    protected Pose getStartPose() {
        return GoalPaths.start;
    }
    @Override
    protected void onInit() {

        f = robot.drivetrain;
        path = new GoalPaths(f);

        isFuturePoseOn = false;
        slowMode = SLOW_MODE;
        SLOW_MODE = 0.35;

        if (Common.isRed != GoalPaths.isPathRed) {
            GoalPaths.isPathRed = !GoalPaths.isPathRed;
            path.mirrorAll();
        }
        Common.robot.shooter.setGoalAlliance();
        path.goal21Build();

        robot.limelight.getLimelight().stop();
        robot.limelight.getLimelight().close();
    }
    @Override
    protected void onRun() {
        shootPreload();
        shootSecond();
        shootGateCycle();
        shootGateCycle();
        shootFirst();
        shootThird();
        goalLeave();
        SLOW_MODE = slowMode;
    }

    private void goalLeave() {
        path.goalLeave21.getPath(0).setTValueConstraint(0.88);
        robot.actionScheduler.addAction(
                new FollowPathAction(f, path.goalLeave21, true));
    }
    private void shootThird() {
        path.thirdIntake21.getPath(2).setTValueConstraint(0.88);
        path.thirdIntake21.getPath(1).setTValueConstraint(0.88);
        path.thirdIntake21.getPath(0).setTValueConstraint(0.88);

        robot.actionScheduler.addAction(
                new Actions.UntilConditionAction(() -> getRuntime() > GoalPaths.LEAVE_TIME,
                        new SequentialAction(
                        new InstantAction(() -> Log.d("AutoGoal", "START_SHOOT_FIRST")),
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.thirdIntake21, 0.3, 0, f, "slow_down_1"), // slow down to intake balls
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.thirdIntake21, 0.01, 2, f, "speed_up_1_post_intake"), // speed up after intake
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel(),
                                                RobotActions.setIntake(0.25, 0)
                                        ),
                                        path.thirdIntake21, 0.01, 2, f, "arm_flywheel_and_turret_1"
                                ),
                                new FollowPathAction(f, path.thirdIntake21, true) // dashes to first 3 balls, starts intake and slows down near halfway points of path
                        ),

                        //shoots first 3 balls
                        RobotActions.shootArtifacts(3, 1.5),

                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_THIRD"))
                )));


        robot.actionScheduler.runBlocking();

    }
    private void shootFirst() {
        path.firstIntake21.getPath(1).setTValueConstraint(0.88);
        path.firstIntake21.getPath(0).setTValueConstraint(0.88);
        robot.actionScheduler.addAction(
                new Actions.UntilConditionAction(() -> getRuntime() > GoalPaths.LEAVE_TIME,
                        new SequentialAction(
                        new InstantAction(() -> Log.d("AutoGoal", "START_SHOOT_FIRST")),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.firstIntake21, .01, 0, f, "speed_up_3"), // speed up to dash to third set of balls
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.firstIntake21, 0.3, 0, f, "intaking_first"), // slow down to intake balls
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel(),
                                                RobotActions.setIntake(0.25, 0)
                                        ),
                                        path.firstIntake21, 0.01, 1, f, "arm_flywheel_and_turret_3"
                                ),
                                new FollowPathAction(f, path.firstIntake21, true)
                        ),

                        RobotActions.shootArtifacts(3, 1.5),
                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_THIRD"))
                )
        ));

        robot.actionScheduler.runBlocking();
    }

    private void shootGateCycle() {
        path.gateCycleIntake21.getPath(0).setTValueConstraint(0.88);
        path.gateCycleShoot21.getPath(0).setTValueConstraint(0.8);
        f.setMaxPower(1);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> Log.d("AutoGoal", "START_GATE_CYCLE")),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(.3)), path.gateCycleIntake21, 0.7, 0, f, "speed_up_2"),
                                new Actions.CallbackAction(
                                        RobotActions.setIntake(1, 0),
                                        path.gateCycleIntake21, 0.7, 0, f, "slow_down_2"),
                                new FollowPathAction(f, path.gateCycleIntake21, true)
                        ),
                        new Actions.UntilConditionAction(
                                () -> robot.shooter.isRobotFullWithBalls(),
                                new SleepAction(2.5) // sleep to let balls roll out of classifier
                        ),
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel(),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.gateCycleShoot21, 0.01, 0, f, "arm_flywheel_and_turret_2"
                                ),
                                new FollowPathAction(f, path.gateCycleShoot21, true)
                        ),

                        new SleepAction(.5),

                        //shoots first 3 balls
                        RobotActions.shootArtifacts(3, 1.5),

                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_GATE"))
                )
        );

        robot.actionScheduler.runBlocking();
    }

    private void shootSecond() {
        path.secondIntake21.getPath(1).setTValueConstraint(0.88);
        path.secondIntake21.getPath(0).setTValueConstraint(0.88);

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> Log.d("AutoGoal", "START_SHOOT_FIRST")),
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.secondIntake21, 0.3, 0, f, "slow_down_1"), // slow down to intake balls
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                new InstantAction(() -> f.setMaxPower(1)),
                                                RobotActions.armTurret(),
                                                RobotActions.armFlywheel(),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.secondIntake21, 0.01, 1, f, "arm_flywheel_and_turret_1"
                                ),
                                new FollowPathAction(f, path.secondIntake21, true) // dashes to first 3 balls, starts intake and slows down near halfway points of path
                        ),

                        //shoots first 3 balls
                        RobotActions.shootArtifacts(3, 1.5),

                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_SECOND"))
                ));


        robot.actionScheduler.runBlocking();

    }

    private void shootPreload() {
        path.shootPreload21.getPath(0).setTValueConstraint(0.88);
        path.shootPreload21.getPath(1).setTValueConstraint(0.88);
        robot.actionScheduler.addAction(

                new SequentialAction( //dashes to line and shoots preloaded 3 balls
                        new InstantAction(() -> Log.d("AutoGoal", "START_SHOOT_PRELOAD")),
                        new ParallelAction(
                                RobotActions.shootArtifacts(3, 4),
                                new Actions.UntilConditionAction(() -> !robot.shooter.isBallPresent(),new ParallelAction(
                                        new Actions.CallbackAction(
                                                RobotActions.emergencyShootArtifacts(),
                                                path.shootPreload21, 0.5, 0, f, "arm_flywheel_and_turret_0"

                                        ),
                                        new Actions.CallbackAction(
                                                new ParallelAction(
                                                        new InstantAction(() -> f.setMaxPower(1)),
                                                        RobotActions.armTurret(),
                                                        RobotActions.armFlywheel()
                                                ),
                                                path.shootPreload21, 0.01, 0, f, "arm_flywheel_and_turret_0"
                                        ),
//                                        new Actions.CallbackAction(
//                                                new InstantAction(() -> isFuturePoseOn = true), path.shootPreload21, 0.2, 0, f, "arm_flywheel_and_turret_0"
//                                        ),
                                        new FollowPathAction(f, path.shootPreload21, true)
                                )
                                )

                        ),
                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_PRELOAD"))
                )
        );

        robot.actionScheduler.runBlocking();
        isFuturePoseOn = false;
    }
}