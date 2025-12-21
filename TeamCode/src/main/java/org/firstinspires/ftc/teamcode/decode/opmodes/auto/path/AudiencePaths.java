package org.firstinspires.ftc.teamcode.decode.opmodes.auto.path;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;

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
            start = new Pose(55.5, 7.25, Math.toRadians(90)),
            shootPreload = new Pose(55.5, 9.25),
            shoot = new Pose(72.2, 26.5),
            leave = new Pose(49.600, 16.200),
            startIntake1 = new Pose(36.1, 26.4),
            endIntake1 = new Pose(13.500, 30.400),
            startIntakeHP1 = new Pose(8.000, 8.500),
            midIntakeHP1 = new Pose(14.300, 8.500),
            endIntakeHP1 = new Pose(10.300, 7.500),
            startIntakeHP2 = new Pose(8.000, 12.200),
            midIntakeHP2 = new Pose(14.700, 12.100),
            endIntakeHP2 = new Pose(10.300, 12.100);

    public static double
            leaveAngle = Math.toRadians(132),
            startAngle = Math.toRadians(90),
            shootAngle = Math.toRadians(-127),
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
        startIntakeHP2 = startIntakeHP2.mirror();
        midIntakeHP2 = midIntakeHP2.mirror();
        endIntakeHP2 = endIntakeHP2.mirror();


        startAngle = mirrorAngleRad(startAngle);
        shootAngle = mirrorAngleRad(shootAngle);
        leaveAngle = mirrorAngleRad(leaveAngle);
        startIntakeAngle = mirrorAngleRad(startIntakeAngle);
        endIntakeAngle = mirrorAngleRad(endIntakeAngle);
        startIntakeAngleHP1 = mirrorAngleRad(startIntakeAngleHP1);
        startIntakeAngleHP2 = mirrorAngleRad(startIntakeAngleHP2);

    }
    public Action moveToBigBalls(List<LLResultTypes.ColorResult> result, Pose robotPose) {
        PathChain path = humanPlayerIntake2;
        if (result.size() > 0) {
            double tx = result.get(0).getTargetXDegrees();
            double wallX;

            wallX = isPathRed ? 141.5 : 2.5;

            double wallDist = Math.abs(robotPose.getX() - wallX);
            double ballDist = Math.tan(Math.toRadians(tx)) * wallDist;

            ballDist = isPathRed ? -ballDist : ballDist;
            wallX += isPathRed ? -10 : 10;

            Pose bigBallPose = new Pose(wallX, robotPose.getY() + ballDist);
            path = f.pathBuilder()
                    .addPath(
                            new BezierLine(f::getPose, bigBallPose)
                    )
                    .setConstantHeadingInterpolation(startIntakeAngleHP1)
                    .build();
            robot.limelight.getLimelight().captureSnapshot("MOVE_TO_BALLS");
            Log.d("MOVE_TO_BALLS_wallX", "" + wallX);
            Log.d("MOVE_TO_BALLS_ballDist", "" + ballDist);
            Log.d("MOVE_TO_BALLS_tx", "" + tx);
            Log.d("MOVE_TO_BALLS_y", "" + (robotPose.getY() + ballDist));

        }
        path.getPath(0).setTValueConstraint(0.8);
        return new ParallelAction(
                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path, .01, 0, f, "speed_up_hp"), // speed up to dash to third set of balls
                new Actions.CallbackAction(
                        new ParallelAction(
                                new InstantAction(() -> f.setMaxPower(1)),
                                RobotActions.setIntake(1, 0)
                        ),
                        path, 0.8, 0, f, "slow_down_hp_2"), // slow down to intake balls
                new Actions.TimedAction(new FollowPathAction(f, path), AudiencePaths.MAX_HP_GOING_MS, "fourthHPAudience")
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
                .build();
        humanPlayerIntake1_5 = f.pathBuilder()
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
                .setConstantHeadingInterpolation(startIntakeAngleHP1)
                .build();
        humanPlayerIntake2 = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(shoot, startIntakeHP2)
                )
                .setConstantHeadingInterpolation(startIntakeAngleHP2)
                .build();
        humanPlayerIntake3 = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(f::getPose, midIntakeHP2)
                )
                .setConstantHeadingInterpolation(startIntakeAngleHP2)
                .build();
        humanPlayerIntake3_5 = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(f::getPose, endIntakeHP2)
                )
                .setConstantHeadingInterpolation(startIntakeAngleHP2)
                .build();
        humanPlayerShoot2 = f.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(f::getPose, shoot)
                )
                .setConstantHeadingInterpolation(startIntakeAngleHP2)
                .build();
        goalLeave = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(shoot, leave)
                )
                .setConstantHeadingInterpolation(leaveAngle)
                .build();
    }
    public PathChain preload;
    public PathChain firstShoot;
    public PathChain humanPlayerIntake0;
    public PathChain humanPlayerIntake1;
    public PathChain humanPlayerIntake1_5;
    public PathChain humanPlayerShoot1;
    public PathChain humanPlayerIntake2;
    public PathChain humanPlayerIntake3;
    public PathChain humanPlayerIntake3_5;
    public PathChain humanPlayerShoot2;
    public PathChain goalLeave;

}
