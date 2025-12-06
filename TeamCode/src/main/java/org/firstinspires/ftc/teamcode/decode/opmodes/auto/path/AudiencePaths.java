package org.firstinspires.ftc.teamcode.decode.opmodes.auto.path;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class AudiencePaths {
    private final Follower f;
    public AudiencePaths(Follower follower) {
        f = follower;
    }
    public static Pose
            start = new Pose(55.5, 7.25, Math.toRadians(90)),
            shoot = new Pose(55.5, 9.3),
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

    public double mirrorAngleRad(double angle) {
        return Math.PI - angle;
    }
    public static boolean isPathRed = false;
    public void audience12Build() {
        firstShoot = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(start, startIntake1)
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
                        new BezierLine(startIntakeHP1, midIntakeHP1)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        humanPlayerShoot1 = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(midIntakeHP1, endIntakeHP1)
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Path 1
                        new BezierLine(endIntakeHP1, shoot)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
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
                        new BezierLine(startIntakeHP2, midIntakeHP2)
                )
                .setConstantHeadingInterpolation(startIntakeAngleHP2)
                .build();
        humanPlayerShoot2 = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(midIntakeHP2, endIntakeHP2)
                )
                .setConstantHeadingInterpolation(startIntakeAngleHP2)
                .addPath(
                        // Path 1
                        new BezierLine(endIntakeHP2, shoot)
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
    public PathChain firstShoot;
    public PathChain humanPlayerIntake0;
    public PathChain humanPlayerIntake1;
    public PathChain humanPlayerShoot1;
    public PathChain humanPlayerIntake2;
    public PathChain humanPlayerIntake3;
    public PathChain humanPlayerShoot2;
    public PathChain goalLeave;

}
