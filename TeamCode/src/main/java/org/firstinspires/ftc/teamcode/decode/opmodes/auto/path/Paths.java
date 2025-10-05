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

    public void buildClose() {
        close = f.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierCurve(
                                new Pose(32.199, 136.621),
                                new Pose(47.627, 113.143),
                                new Pose(41.366, 102.186)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-127))
                .addPath(
                        // Path 2
                        new BezierLine(new Pose(41.366, 102.186), new Pose(33.317, 91.453))
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(33.317, 91.453), new Pose(17.665, 88.547))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-127), Math.toRadians(-161))
                .addPath(
                        // Path 4
                        new BezierLine(new Pose(17.665, 88.547), new Pose(44.720, 97.938))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        // Path 5
                        new BezierLine(new Pose(44.720, 97.938), new Pose(32.646, 67.975))
                )
                .setConstantHeadingInterpolation(Math.toRadians(240))
                .addPath(
                        // Path 6
                        new BezierLine(new Pose(32.646, 67.975), new Pose(15.205, 65.068))
                )
                .setLinearHeadingInterpolation(Math.toRadians(240), Math.toRadians(-157))
                .addPath(
                        // Path 7
                        new BezierLine(new Pose(15.205, 65.068), new Pose(59.702, 84.075))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        // Path 8
                        new BezierLine(new Pose(59.702, 84.075), new Pose(34.211, 40.248))
                )
                .setConstantHeadingInterpolation(Math.toRadians(-122))
                .addPath(
                        // Path 9
                        new BezierLine(new Pose(34.211, 40.248), new Pose(17.441, 42.484))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-122), Math.toRadians(-150))
                .addPath(
                        // Path 10
                        new BezierLine(new Pose(17.441, 42.484), new Pose(70.658, 73.565))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }


    public PathChain close;
}
