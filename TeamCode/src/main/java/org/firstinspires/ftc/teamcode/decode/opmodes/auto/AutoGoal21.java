package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.GoalPaths.shoot21;
import static org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.GoalPaths.shootThird21;
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
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.GoalPaths;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.LaunchZone;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.decode.util.Actions;
import org.firstinspires.ftc.teamcode.decode.util.FollowPathAction;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "AutoGoal21", preselectTeleOp = "Main TeleOp")

public class AutoGoal21 extends AbstractAuto{
    private GoalPaths path;

    public static double
            FIRST_INTAKE_BRAKING_STRENGTH = 1,
            FIRST_INTAKE_BRAKING_START = 1,
            THIRD_INTAKE_BRAKING_STRENGTH = 2,
            THIRD_INTAKE_BRAKING_START = 2.7,
            OFFSETY_CYCLE_ONE = 0,
            OFFSETY_CYCLE_TWO = -.1,
            OFFSETY_CYCLE_THREE = -0.1,
            OFFSETX_CYCLE_ONE = 0,
            OFFSETX_CYCLE_TWO = -0.2,
            OFFSETX_CYCLE_THREE = -0.1;

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
        path.goal21Build();

//        f.setConstraints(Constants.pathConstraints);
//        robot.shooter.turret.run();
        robot.limelight.getLimelight().stop();
        robot.limelight.getLimelight().close();
    }
    @Override
    protected void onRun() {
        shootPreload();
        shootSecond();
        shootGateCycle(OFFSETY_CYCLE_ONE, OFFSETX_CYCLE_ONE,1.5, 25.85, false);
        shootGateCycle(OFFSETY_CYCLE_TWO, OFFSETX_CYCLE_TWO,1.5, 25.85, false);
        shootFirst();
        shootGateCycle(OFFSETY_CYCLE_THREE,OFFSETX_CYCLE_THREE,1.5, 25.85, false);
        shootThird();
        goalLeave();
    }

    void goalLeave() {
        path.goalLeave21.getPath(0).setTValueConstraint(0.88);
        robot.actionScheduler.addAction(
                new FollowPathAction(f, path.goalLeave21, true));
    }
    void shootThird() {
        path.thirdIntake21.getPath(2).setTValueConstraint(.8);
        path.thirdIntake21.getPath(1).setTValueConstraint(0.88);
        path.thirdIntake21.getPath(0).setTValueConstraint(0.88);

        path.thirdIntake21.getPath(2).setBrakingStrength(THIRD_INTAKE_BRAKING_STRENGTH);
        path.thirdIntake21.getPath(2).setBrakingStart(THIRD_INTAKE_BRAKING_START);

        path.thirdIntake21.getPath(2).setVelocityConstraint(0.5);

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
                                                        RobotActions.setIntake(0.25, 0),
                                                        new InstantAction(() ->         robot.shooter.feeder.isGateEnabled = false)
                                                ),
                                                path.thirdIntake21, 0.1, 2, f, "arm_flywheel_and_turret_1"
                                        ),

//                                        new Actions.CallbackAction(
//                                                new ParallelAction(
//                                                        new InstantAction(() -> f.setMaxPower(.3))
//                                                        ),
//                                                path.thirdIntake21, 0.5, 2, f, "arm_flywheel_and_turret_1"
//                                        ),

                                        new Actions.CallbackAction(
                                                new ParallelAction(
//                                                        new InstantAction(() -> f.setMaxPower(.3)),
                                                        new InstantAction(() -> {
                                                            PathConstraints constraints = f.getConstraints();
                                                            constraints.setBrakingStart(5);
                                                            constraints.setBrakingStrength(7);
                                                            f.setConstraints(constraints);
                                                        })
                                                ),
                                                path.thirdIntake21, 0.5, 2, f, "arm_flywheel_and_turret_1"
                                        ),


                                        new Actions.CallbackAction(
                                                new ParallelAction(
//                                                        new InstantAction(() -> f.setMaxPower(.3)),
                                                        new InstantAction(() -> {
                                                            PathConstraints constraints = f.getConstraints();
                                                            constraints.setBrakingStart(.6);
                                                            constraints.setBrakingStrength(.05);
                                                            f.setConstraints(constraints);
                                                        })
                                                ),
                                                path.thirdIntake21, 0.4, 2, f, "arm_flywheel_and_turret_1"
                                        ),

//                                        new Actions.CallbackAction(
//                                                new InstantAction(()->f.setMaxPower(.2)),
//                                                path.thirdIntake21, .5, 2, f, "arm_flywheel_and_turret_1"
//                                        ),

                                        new Actions.CallbackAction(
                                                new SequentialAction(
                                                        new Actions.UntilConditionAction(() -> LaunchZone.getCurrentZone(f.getPose()) == LaunchZone.NEAR, new SleepAction(10)),
//                                                        new SleepAction(1),
//                                                        new SleepAction(.3),
                                                        new ParallelAction(
                                                                RobotActions.shootArtifacts(3, 1),
//                                                                new SequentialAction(
//                                                                        new SleepAction(.01),
                                                                        RobotActions.emergencyShootArtifacts()
//                                                                )
                                                        )
                                                ) ,
                                                path.thirdIntake21, .01, 2, f, "arm_flywheel_and_turret_1"
                                        ),
                                        new FollowPathAction(f, path.thirdIntake21, true) // dashes to first 3 balls, starts intake and slows down near halfway points of path
                                ),

                                //shoots first 3 balls
//                                RobotActions.shootArtifacts(3, 1),
//                                RobotActions.emergencyShootArtifacts(),
                                new InstantAction(() -> f.setMaxPower(0)),

                                new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_THIRD"))
                        )));

        isFuturePoseOn = true;
        robot.shooter.feeder.isGateEnabled = true;
        robot.shooter.setDontMoveGoalDown(true);
        robot.actionScheduler.runBlocking();
        robot.shooter.feeder.isGateEnabled = false;
        robot.shooter.setDontMoveGoalDown(false);

        isFuturePoseOn = false;

    }
    void shootFirst() {
        path.firstIntake21.getPath(1).setTValueConstraint(.8);
        path.firstIntake21.getPath(0).setTValueConstraint(0.88);

        path.firstIntake21.getPath(1).setBrakingStrength(FIRST_INTAKE_BRAKING_STRENGTH);
        path.firstIntake21.getPath(1).setBrakingStart(FIRST_INTAKE_BRAKING_START);

//        path.firstIntake21.getPath(1).setVelocityConstraint(.02);


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
                                        new Actions.CallbackAction(
                                                new ParallelAction(
                                                        new InstantAction(() ->         robot.shooter.feeder.isGateEnabled = false)
                                                ),
                                                path.firstIntake21, 0.1, 1, f, "arm_flywheel_and_turret_1"
                                        ),
//                                        new Actions.CallbackAction(
//                                                new ParallelAction(
////                                                        new InstantAction(() -> f.setMaxPower(.5)),
//                                                        new InstantAction(() -> {
//                                                            PathConstraints constraints = f.getConstraints();
//                                                            constraints.setBrakingStart(.3);
//                                                            constraints.setBrakingStrength(1);
//                                                            f.setConstraints(constraints);
//                                                        })
//                                                ),
//                                                path.firstIntake21, 0.6, 1, f, "arm_flywheel_and_turret_1"
//                                        ),

                                        new Actions.CallbackAction(
                                                new ParallelAction(
                                                        RobotActions.shootArtifacts(3, 1.5),
                                                        new SequentialAction(
                                                                new SleepAction(.1),
                                                                RobotActions.emergencyShootArtifacts()
                                                        )
                                                ),
                                                path.firstIntake21, 0.78, 1, f, "arm_flywheel_and_turret_2"
                                        ),
                                        new FollowPathAction(f, path.firstIntake21, true)
                                ),

//                                RobotActions.shootArtifacts(3, 1),
//                                RobotActions.emergencyShootArtifacts(),
                                new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_THIRD"))
                        )
                ));


        robot.shooter.feeder.isGateEnabled = true;
        isFuturePoseOn = false;
        robot.actionScheduler.runBlocking();
        robot.shooter.feeder.isGateEnabled = false;
        isFuturePoseOn = true;
        f.setMaxPower(1);
    }

    void shootGateCycle(double offsetY, double offsetX, double waitTime, double slowDownX, boolean leave) {
        Pose intakeGateCycle21 = GoalPaths.intakeGateCycle21.withY(GoalPaths.intakeGateCycle21.getY() + offsetY).withX(GoalPaths.intakeGateCycle21.getX() + offsetX * (isRed ? 1 : -1));

        PathChain gateCycleIntake21 = f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shoot21,
                                GoalPaths.gateCycleControl21,
                                intakeGateCycle21
                        )
                )
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                .3,
                                HeadingInterpolator.tangent
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                .3,
                                1,
                                HeadingInterpolator.constant(GoalPaths.gateCycleIntakeAngle)
                        )
                ))
                .build();

        PathChain gateCycleShoot21 = f.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakeGateCycle21,
                                leave ? shootThird21 : shoot21
                        )
                )

//                .setHeadingInterpolation(HeadingInterpolator.piecewise(
//                        new HeadingInterpolator.PiecewiseNode(
//                                0,
//                                .3,
//                                HeadingInterpolator.constant((Math.PI+gateCycleIntakeAngle)%(2*Math.PI))
//                        ),
//                        new HeadingInterpolator.PiecewiseNode(
//                                0.3,
//                                1,
//                                HeadingInterpolator.tangent
//                        )
//                ))
                .setTangentHeadingInterpolation()
                .setReversed().build();

        gateCycleIntake21.getPath(0).setTValueConstraint(0.94);
        gateCycleIntake21.getPath(0).setHeadingConstraint(0.0005);
        gateCycleIntake21.getPath(0).setTranslationalConstraint(0.001);
        gateCycleShoot21.getPath(0).setTValueConstraint(.92);
        gateCycleShoot21.getPath(0).setBrakingStart(1);
        gateCycleShoot21.getPath(0).setBrakingStrength(1.4);

//        gateCycleIntake21.getPath(0).setVelocityConstraint(.02);
//        path.gateCycleShoot21.getPath(0).setVelocityConstraint(.02);


        f.setMaxPower(1);
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> Log.d("AutoGoal", "START_GATE_CYCLE")),
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        new SequentialAction(
                                                new Actions.UntilConditionAction(() -> isRed ? f.getPose().getX() > 144-slowDownX : f.getPose().getX() < slowDownX, new SleepAction(3)),
                                                new InstantAction(() -> f.setMaxPower(.3))
                                        ),

                                        gateCycleIntake21, 0.1, 0, f, "speed_up_2"),
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
                                        new SleepAction(waitTime) // sleep to let balls roll out of classifier
                                        )
                        ),
                        RobotActions.closeGate() ,
                        new InstantAction(() -> f.setMaxPower(1)),
                        new ParallelAction(
                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                RobotActions.armTurret(),
//                                                RobotActions.armFlywheel(),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        gateCycleShoot21, 0.01, 0, f, "arm_flywheel_and_turret_2"
                                ),

                                new Actions.CallbackAction(
                                        leave ?
                                            new SequentialAction(
                                                new Actions.UntilConditionAction(()->LaunchZone.getCurrentZone(f.getPose())==LaunchZone.NEAR, new SleepAction(10)),
                                                new ParallelAction(
                                                        RobotActions.shootArtifacts(3, 1.5),
                                                        new SequentialAction(
                                                                new SleepAction(.1),
                                                                RobotActions.emergencyShootArtifacts()
                                                        )
                                                )
                                            )
                                                :
                                            new ParallelAction(
                                                    RobotActions.shootArtifacts(3, 1.5),
                                                    new SequentialAction(
                                                            new SleepAction(.1),
                                                            RobotActions.emergencyShootArtifacts()
                                                    )
                                            ),
                                        gateCycleShoot21, leave ? .1:0.92, 0, f, "arm_flywheel_and_turret_2"
                                ),
                                new FollowPathAction(f, gateCycleShoot21, true)
                        ),

//                        new SleepAction(.2),

                        //shoots first 3 balls
//                        RobotActions.shootArtifacts(3, 2),
//
                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_GATE"))
                )
        );


        robot.shooter.feeder.isGateEnabled = false;
        isFuturePoseOn = true;
        robot.actionScheduler.runBlocking();
        isFuturePoseOn = true;
        robot.shooter.feeder.isGateEnabled = false;
    }

    void shootSecond() {
        path.secondIntake21.getPath(1).setTValueConstraint(0.88);
        path.secondIntake21.getPath(0).setTValueConstraint(0.88);

//        path.secondIntake21.getPath(1).setVelocityConstraint(.02);


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
//                                                RobotActions.armFlywheel(),
                                                RobotActions.setIntake(1, 0)
                                        ),
                                        path.secondIntake21, 0.01, 1, f, "arm_flywheel_and_turret_1"
                                ),

                                new Actions.CallbackAction(
                                        new ParallelAction(
                                                RobotActions.shootArtifacts(3, 1.5)
//                                                new SequentialAction(
//                                                        new SleepAction(.1),
//                                                        RobotActions.emergencyShootArtifacts()
//                                                )
                                        ),
                                        path.secondIntake21, 0.83, 1, f, "arm_flywheel_and_turret_2"
                                ),

                                new FollowPathAction(f, path.secondIntake21, true) // dashes to first 3 balls, starts intake and slows down near halfway points of path
                        ),

                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_SECOND"))
                ));



        robot.shooter.feeder.isGateEnabled = false;
        isFuturePoseOn = true;
        robot.actionScheduler.runBlocking();
        isFuturePoseOn = true;
        robot.shooter.feeder.isGateEnabled = false;
    }

    void shootPreload() {
        path.shootPreload21.getPath(0).setTValueConstraint(0.88);
        path.shootPreload21.getPath(1).setTValueConstraint(0.88);
//        path.shootPreload21.getPath(1).setVelocityConstraint(.02);

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
//                                                        new InstantAction(() -> isFuturePoseOn = true), path.shootPreload21, 0.4, 0, f, "arm_flywheel_and_turret_0"
//                                                ),

                                                new Actions.CallbackAction(
                                                        new SequentialAction(
//                                                                new InstantAction(() -> f.setMaxPower(.5)),
//                                                                new InstantAction(() -> f.pausePathFollowing()),
//                                                                new SleepAction(.1),
                                                                new ParallelAction(
                                                                        RobotActions.shootArtifacts(3, 2, false),
                                                                        new SequentialAction(
                                                                                new SleepAction(.1),
                                                                                RobotActions.emergencyShootArtifacts()
                                                                        )
                                                                )
//                                                                new InstantAction(() -> f.resumePathFollowing())

                                                                )
                                                        , path.shootPreload21, .5, 0, f, "arm_flywheel_and_turret_0"
                                                ),
                                                new FollowPathAction(f, path.shootPreload21, true)

                                        )),


                                        new InstantAction(() -> f.breakFollowing()),
                                        new InstantAction(() -> f.setMaxPower(1))
                                )

                        ),
                        new InstantAction(() -> Log.d("AutoGoal", "END_SHOOT_PRELOAD"))
                )
        );

        isFuturePoseOn = true;
        robot.shooter.feeder.isGateEnabled = false;
        robot.actionScheduler.runBlocking();
        robot.shooter.feeder.isGateEnabled = false;
        f.breakFollowing();
    }
}