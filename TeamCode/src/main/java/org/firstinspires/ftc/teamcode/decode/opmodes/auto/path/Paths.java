package org.firstinspires.ftc.teamcode.decode.opmodes.auto.path;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    private final Follower f;
    public Paths(Follower follower) {
        f = follower;
    }

    public static Pose
            control0 = new Pose(47.6, 113.1),
            start = new Pose(32.2, 136.6),
            end0 = new Pose(41.4, 102.2),
            end1 = new Pose(33.3, 91.5),
            end2 = new Pose(17.7, 88.5),
            end3 = new Pose(44.7, 97.9),
            end4 = new Pose(32.7, 68.0),
            end5 = new Pose(15.2, 65.1),
            end6 = new Pose(59.7, 84.1),
            end7 = new Pose(34.2, 40.3),
            end8 = new Pose(17.4, 42.5),
            end9 = new Pose(70.7, 73.6);
    public static double
            startAngle = Math.toRadians(270),
            endAngle0 = Math.toRadians(-127),
            endAngle1 = Math.toRadians(-161),
            endAngle2 = Math.toRadians(240),
            endAngle3 = Math.toRadians(-157),
            endAngle4 = Math.toRadians(-122),
            endAngle5 = Math.toRadians(-150);


    public void buildClose() {
        close = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierCurve(
                                start,
                                control0,
                                end0
                        )
                )
                .setLinearHeadingInterpolation(startAngle, endAngle0)
                .addPath(
                        // Path 1
                        new BezierLine(end0, end1)
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Path 2
                        new BezierLine(end1, end2)
                )
                .setLinearHeadingInterpolation(endAngle0, endAngle1)
                .addPath(
                        // Path 3
                        new BezierLine(end2, end3)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        // Path 4
                        new BezierLine(end3, end4)
                )
                .setConstantHeadingInterpolation(endAngle2)
                .addPath(
                        // Path 5
                        new BezierLine(end4, end5)
                )
                .setLinearHeadingInterpolation(endAngle2, endAngle3)
                .addPath(
                        // Path 6
                        new BezierLine(end5, end6)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        // Path 7
                        new BezierLine(end6, end7)
                )
                .setConstantHeadingInterpolation(endAngle4)
                .addPath(
                        // Path 8
                        new BezierLine(end7, end8)
                )
                .setLinearHeadingInterpolation(endAngle4, endAngle5)
                .addPath(
                        // Path 9
                        new BezierLine(end8, end9)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }


    public PathChain close;
}
