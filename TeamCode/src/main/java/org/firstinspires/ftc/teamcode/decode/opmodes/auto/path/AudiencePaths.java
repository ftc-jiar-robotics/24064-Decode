package org.firstinspires.ftc.teamcode.decode.opmodes.auto.path;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.decode.util.Actions;
import org.firstinspires.ftc.teamcode.decode.util.FollowPathAction;

public class AudiencePaths {
    private final Follower f;
    public AudiencePaths(Follower follower) {
        f = follower;
    }

    public static long
            MAX_HP_TIME_MS = 567,
            MAX_HP_GOING_MS = 6000;
    public static double LEAVE_TIME = 29.5;

    public static Pose

            start = Common.BLUE_SMALL_TRIANGLE,
            shootPreload = new Pose(55.5, 11.25),
            shoot = new Pose(60.2, 19.4),
            leave = new Pose(40.1, 24.9),
            startIntake1 = new Pose(36.1, 28.4),
            endIntake1 = new Pose(13.500, 30.400),
            startIntakeHP1 = new Pose(8.000, 8.500),
            midIntakeHP1 = new Pose(14.300, 8.500),
            endIntakeHP1 = new Pose(10.300, 8.500);

    public static double
            startAngle = Math.toRadians(90),
            shootAngle = Math.toRadians(167),
            startIntakeAngle = Math.toRadians(153),
            endIntakeAngle = Math.toRadians(153),
            startIntakeAngleHP1 = Math.toRadians(180),
            startIntakeAngleHP2 = Math.toRadians(175);

    public void mirrorAll() {
        shootPreload = shootPreload.mirror();
        start = start.mirror();
        shoot = shoot.mirror();
        leave = leave.mirror();
        startIntake1 = startIntake1.mirror();
        endIntake1 = endIntake1.mirror();
        startIntakeHP1 = startIntakeHP1.mirror();
        midIntakeHP1 = midIntakeHP1.mirror();
        endIntakeHP1 = endIntakeHP1.mirror();

        startAngle = mirrorAngleRad(startAngle);
        shootAngle = mirrorAngleRad(shootAngle);
        startIntakeAngle = mirrorAngleRad(startIntakeAngle);
        endIntakeAngle = mirrorAngleRad(endIntakeAngle);
        startIntakeAngleHP1 = mirrorAngleRad(startIntakeAngleHP1);
        startIntakeAngleHP2 = mirrorAngleRad(startIntakeAngleHP2);

    }
    public Action retrieveBigBalls(List<LLResultTypes.ColorResult> result, Pose robotPose) {
        boolean isArtifactFound = !result.isEmpty();
        if (!isArtifactFound) {
            robot.limelight.getLimelight().pipelineSwitch(1);
            robot.limelight.update();
            result = robot.limelight.getColorResult();
            isArtifactFound = !result.isEmpty();
            robot.limelight.getLimelight().pipelineSwitch(2);
        }

        PathChain path = humanPlayerIntake0;
        PathChain pathBack = humanPlayerIntake1;
        PathChain pathShoot = humanPlayerShoot1;

        robot.limelight.getLimelight().captureSnapshot("MOVE_TO_BALLS");
        if (isArtifactFound) {
            double tx = result.get(0).getTargetXDegrees();
            tx = (isPathRed ? tx : tx - 180) - Math.toDegrees(robotPose.getHeading());
            double wallX;

            wallX = isPathRed ? 141.5 : 2.5;

            double wallDist = Math.abs(robotPose.getX() - wallX);
            double ballDist = Math.tan(Math.toRadians(tx)) * wallDist;

            ballDist = isPathRed ? -ballDist : ballDist;
            wallX += isPathRed ? -10 : 10;

            Pose bigBallPose = new Pose(wallX, Math.max(endIntakeHP1.getY(),robotPose.getY() + ballDist));

            path = f.pathBuilder()
                    .addPath(
                            new BezierLine(f::getPose, bigBallPose)
                    )
                    .setConstantHeadingInterpolation(startIntakeAngleHP1)
                    .build();

            pathBack = f.pathBuilder()
                    .addPath(
                            // Path 0
                            new BezierLine(f::getPose, new Pose(bigBallPose.getX() + (isPathRed ? -4 : 4), bigBallPose.getY()))
                    )
                    .setConstantHeadingInterpolation(startIntakeAngleHP1)
                    .addPath(
                            // Path 1
                            new BezierLine(f::getPose, bigBallPose)
                    )
                    .setConstantHeadingInterpolation(startIntakeAngleHP1)
                    .build();

            pathShoot = f.pathBuilder()
                    .addPath(
                            // Path 0
                            new BezierLine(bigBallPose, shoot)
                    )
                    .setConstantHeadingInterpolation(shootAngle)
                    .build();

            Log.d("MOVE_TO_BALLS_wallX", "" + wallX);
            Log.d("MOVE_TO_BALLS_ballDist", "" + ballDist);
            Log.d("MOVE_TO_BALLS_tx", "" + tx);
            Log.d("MOVE_TO_BALLS_y", "" + (robotPose.getY() + ballDist));
            Log.d("MOVE_TO_BALLS_robotHeading", "" + robotPose.getHeading());

        }
        path.getPath(0).setTValueConstraint(0.8);
        pathBack.getPath(0).setTValueConstraint(0.75);
        pathBack.getPath(1).setTValueConstraint(0.8);
        return new SequentialAction(
                new ParallelAction(
                        new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path, .01, 0, f, "speed_up_hp"), // speed up to dash to third set of balls
                        new Actions.CallbackAction(
                                RobotActions.setIntake(1, 0),
                                path, 0.5, 0, f, "slow_down_hp_2"), // slow down to intake balls
                        new Actions.TimedAction(new FollowPathAction(f, path), AudiencePaths.MAX_HP_GOING_MS, "fourthHPAudience")
                ),
                new SleepAction(0.1),
                new Actions.TimedAction(new FollowPathAction(f, pathBack.getPath(0)), AudiencePaths.MAX_HP_TIME_MS, "fifthHPAudience"),
                new SleepAction(0.1),
                new Actions.TimedAction(new FollowPathAction(f, pathBack.getPath(1)), AudiencePaths.MAX_HP_TIME_MS, "sixthHPAudience"),
                new ParallelAction(
                        new Actions.CallbackAction(
                                new ParallelAction(
                                        new InstantAction(() -> f.setMaxPower(1)),
                                        RobotActions.armTurret(),
                                        RobotActions.armFlywheel()
                                ),
                                pathShoot, 0.01, 0, f, "arm_flywheel_and_turret_hp_2"
                        ),
                        new Actions.CallbackAction(
                                RobotActions.setIntake(0, 0),
                                pathShoot, 0.3, 0, f, "stop_intake"
                        ),
                        new FollowPathAction(f, pathShoot, true)
                )
        );
    }
    public double mirrorAngleRad(double angle) {
        return Math.PI - angle;
    }
    public static boolean isPathRed = false;
    public void audience12Build() {
        preload = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(start, shootPreload)
                )
                .setConstantHeadingInterpolation(startAngle)
                .build();

        firstShoot = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(shootPreload, startIntake1)
                )
                .setLinearHeadingInterpolation(startAngle, startIntakeAngle)
                .addPath(
                        // Path 1
                        new BezierLine(startIntake1, endIntake1)
                )
                .setLinearHeadingInterpolation(startIntakeAngle, endIntakeAngle)
                .addPath(
                        // Path 2
                        new BezierLine(endIntake1, shoot)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        humanPlayerIntake0 = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(shoot, startIntakeHP1)
                )
                .setConstantHeadingInterpolation(startIntakeAngleHP1)
                .build();
        humanPlayerIntake1 = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(f::getPose, midIntakeHP1)
                )
                .setConstantHeadingInterpolation(startIntakeAngleHP1)
                .addPath(
                        // Path 0
                        new BezierLine(f::getPose, endIntakeHP1)
                )
                .setConstantHeadingInterpolation(startIntakeAngleHP1)
                .build();

        humanPlayerShoot1 = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(f::getPose, shoot)
                )
                .setConstantHeadingInterpolation(shootAngle)
                .build();
        audienceLeave = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(shoot, leave)
                )
                .setConstantHeadingInterpolation(shootAngle)
                .build();
    }
    public PathChain preload;
    public PathChain firstShoot;
    public PathChain humanPlayerIntake0;
    public PathChain humanPlayerIntake1;
    public PathChain humanPlayerShoot1;
    public PathChain audienceLeave;

}
