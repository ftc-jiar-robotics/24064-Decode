package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.SLOW_MODE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFuturePoseOn;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isRed;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.AudiencePaths;
import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.GoalPaths;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.decode.util.Actions;
import org.firstinspires.ftc.teamcode.decode.util.FollowPathAction;

@Configurable
@Autonomous(name = "AutoGoal21", preselectTeleOp = "Main TeleOp")

public class AutoGoal21 extends AbstractAuto{
    private GoalPaths path;

    public static double
            FIRST_INTAKE_BRAKING_STRENGTH = 2,
            FIRST_INTAKE_BRAKING_START = 3,
            THIRD_INTAKE_BRAKING_STRENGTH = 3,
            THIRD_INTAKE_BRAKING_START = 3,
            OFFSETY_CYCLE_ONE = 0,
            OFFSETY_CYCLE_TWO = 0,
            OFFSETY_CYCLE_THREE = 0.3,
            OFFSETX_CYCLE_ONE = 0,
            OFFSETX_CYCLE_TWO = .3,
            OFFSETX_CYCLE_THREE = .3;

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
        path.goal21Build();

        robot.limelight.getLimelight().stop();
        robot.limelight.getLimelight().close();
    }
    @Override
    protected void onRun() {
        shootPreload();
        shootSecond();
        shootGateCycle(OFFSETY_CYCLE_ONE, OFFSETX_CYCLE_ONE);
        shootGateCycle(OFFSETY_CYCLE_TWO, OFFSETX_CYCLE_TWO);
        shootGateCycle(OFFSETY_CYCLE_THREE,OFFSETX_CYCLE_THREE);
        shootFirst();
        shootThird();
        goalLeave();
    }

    private void goalLeave() {
        path.goalLeave21.getPath(0).setTValueConstraint(0.88);
        robot.actionScheduler.addAction(
                new FollowPathAction(f, path.goalLeave21, true));
    }
    private void shootThird() {
        path.thirdIntake21.getPath(2).setTValueConstraint(.99);
        path.thirdIntake21.getPath(1).setTValueConstraint(0.88);
        path.thirdIntake21.getPath(0).setTValueConstraint(0.88);

        path.thirdIntake21.getPath(2).setBrakingStrength(THIRD_INTAKE_BRAKING_STRENGTH);
        path.thirdIntake21.getPath(2).setBrakingStart(THIRD_INTAKE_BRAKING_START);

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
                                                path.thirdIntake21, 0.1, 2, f, "arm_flywheel_and_turret_1"
                                        ),
                                        new FollowPathAction(f, path.thirdIntake21, true) // dashes to first 3 balls, starts intake and slows down near halfway points of path
                                ),

                                new SleepAction(.5),

                                //shoots first 3 balls
                                RobotActions.shootArtifacts(3, 1),

                                new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_THIRD"))
                        )));

//        robot.shooter.feeder.isGateEnabled = false;
        robot.actionScheduler.runBlocking();
//        robot.shooter.feeder.isGateEnabled = true;

    }
    private void shootFirst() {
        path.firstIntake21.getPath(1).setTValueConstraint(.99);
        path.firstIntake21.getPath(0).setTValueConstraint(0.88);

        path.firstIntake21.getPath(1).setBrakingStrength(FIRST_INTAKE_BRAKING_STRENGTH);
        path.firstIntake21.getPath(1).setBrakingStart(FIRST_INTAKE_BRAKING_START);

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
                                                path.firstIntake21, 0.2, 0, f, "intaking_first"), // slow down to intake balls
                                        new Actions.CallbackAction(
                                                new ParallelAction(
                                                        RobotActions.armTurret(),
                                                        RobotActions.armFlywheel(),
                                                        RobotActions.setIntake(0.25, 0)
                                                ),
                                                path.firstIntake21, 0.2, 1, f, "arm_flywheel_and_turret_3"
                                        ),
                                        new FollowPathAction(f, path.firstIntake21, true)
                                ),

                                RobotActions.shootArtifacts(3, 2),
                                new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_THIRD"))
                        )
                ));

        robot.actionScheduler.runBlocking();
    }

    private void shootGateCycle(double offsetY, double offsetX) {
        Pose intakeGateCycle21 = GoalPaths.intakeGateCycle21.withY(GoalPaths.intakeGateCycle21.getY() + offsetY).withX(GoalPaths.intakeGateCycle21.getX() + offsetX * (isRed ? 1 : -1));

        PathChain gateCycleIntake21 = f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                GoalPaths.shoot21,
                                GoalPaths.gateCycleControl21,
                                intakeGateCycle21
                        )
                )
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                .2,
                                HeadingInterpolator.tangent
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                0.2,
                                1,
                                HeadingInterpolator.constant(GoalPaths.gateCycleIntakeAngle)
                        )
                ))
                .build();

        gateCycleIntake21.getPath(0).setTValueConstraint(0.94);
        gateCycleIntake21.getPath(0).setHeadingConstraint(0.00077);
        gateCycleIntake21.getPath(0).setTranslationalConstraint(0.01);
        path.gateCycleShoot21.getPath(0).setTValueConstraint(.95);

        f.setMaxPower(1);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> Log.d("AutoGoal", "START_GATE_CYCLE")),
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(.25)), gateCycleIntake21, 0.65, 0, f, "speed_up_2"),
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                RobotActions.setIntake(1, 0),
                                                RobotActions.openGate()
                                        ),
                                        gateCycleIntake21, 0.5, 0, f, "slow_down_2"),
                                new FollowPathAction(f, gateCycleIntake21, true)
                        ),
                        new Actions.UntilConditionAction(
                                () -> robot.shooter.isRobotFullWithBalls(),
                                new SequentialAction(
                                        new InstantAction(() -> f.setMaxPower(1)),
                                        new SleepAction(1.5) // sleep to let balls roll out of classifier
                                )
                        ),
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                RobotActions.closeGate(),
                                                RobotActions.armTurret(),
//                                                RobotActions.armFlywheel(),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.gateCycleShoot21, 0.01, 0, f, "arm_flywheel_and_turret_2"
                                ),

                                new Actions.CallbackAction(
                                        new SequentialAction(
                                                RobotActions.shootArtifacts(3, 2),
                                                RobotActions.emergencyShootArtifacts()
                                        ),
                                        path.gateCycleShoot21, 0.9, 0, f, "arm_flywheel_and_turret_2"
                                ),
                                new FollowPathAction(f, path.gateCycleShoot21, true)
                        ),

//                        new SleepAction(.2),

                        //shoots first 3 balls
//                        RobotActions.shootArtifacts(3, 2),
//
                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_GATE"))
                )
        );


//        robot.shooter.feeder.isGateEnabled = false;
        robot.actionScheduler.runBlocking();
//        robot.shooter.feeder.isGateEnabled = true;
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
                        RobotActions.shootArtifacts(3, 1),

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
                                new SequentialAction(
                                        new Actions.UntilConditionAction(() -> !robot.shooter.isBallPresent(),new ParallelAction(
                                                new Actions.CallbackAction(
                                                        RobotActions.armTurret(),
                                                        path.shootPreload21, 0.2, 0, f, "arm_turret_0"
                                                ),
                                                new Actions.CallbackAction(
                                                        RobotActions.armFlywheel(),
                                                        path.shootPreload21, 0.01, 0, f, "arm_flywheel_0"
                                                ),
//                                                new Actions.CallbackAction(
//                                                        new InstantAction(() -> isFuturePoseOn = true), path.shootPreload21, 0.2, 0, f, "arm_flywheel_and_turret_0"
//                                                ),
                                                new FollowPathAction(f, path.shootPreload21, true)

                                        )),
                                        RobotActions.shootArtifacts(3, 2, false),
                                        new InstantAction(() -> f.setMaxPower(1))
                                )

                        ),
                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_PRELOAD"))
                )
        );

        robot.actionScheduler.runBlocking();
        isFuturePoseOn = false;
    }
}