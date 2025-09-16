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
    public PathBuilder builder;

    public CustomPaths(Follower drivetrain) {
        builder = new PathBuilder(drivetrain);
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

        straightLine = builder
                .addPath(new BezierLine(new Pose(8.291, 65.000), endPoints[0]))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        curveRight = builder
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
