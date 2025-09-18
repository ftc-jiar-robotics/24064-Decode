package org.firstinspires.ftc.teamcode.decode.opmodes.auto.path;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

@Config
public class CustomPaths {
    public Follower f;

    public CustomPaths(Follower follower) {
        f = follower;
    }

    public void build() {
        endPoints = new Pose[]{
                new Pose(30.000, 80.000),
                new Pose(36.000, 35.000)
        };

        controlPoints = new Pose[]{
                new Pose(48.000, 80.000),
                new Pose(25.000, 56.000)
        };

        straightLine = f.pathBuilder()
                .addPath(new BezierLine(new Pose(8.291, 80), endPoints[0]))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        curveRight = f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                endPoints[0],
                                controlPoints[0],
                                controlPoints[1],
                                endPoints[1]
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public Pose[] controlPoints;
    public Pose[] endPoints;
    public PathChain straightLine;
    public PathChain curveRight;
}
