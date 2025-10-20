package org.firstinspires.ftc.teamcode.decode.opmodes.auto.path;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Configurable
public class Paths {
    private final Follower f;
    public Paths(Follower follower) {
        f = follower;
    }

    public static boolean isPathRed = false;
    // TODO put all poses/heading in respective named list & mirror thru list

    public static Pose
            control0 = new Pose(47.6, 113.1),
            start = new Pose(30.5, 135.5, Math.toRadians(270)),
            end0 = new Pose(51.0, 98.0),
            end1 = new Pose(33.3, 91.5),
            end2 = new Pose(17.7, 88.5),
            end3 = new Pose(51.0, 98.0),
            end4 = new Pose(32.7, 68.0),
            end5 = new Pose(15.2, 65.1),
            end6 = new Pose(51.0, 98.0),
            end7 = new Pose(34.2, 40.3),
            end8 = new Pose(15.4, 42.5),
            end9 = new Pose(51.0, 98.0);

    public static double
            startAngle = Math.toRadians(270),
            endAngle0 = Math.toRadians(-127),
            endAngle1 = Math.toRadians(-161),
            endAngle2 = Math.toRadians(240),
            endAngle3 = Math.toRadians(-157),
            endAngle4 = Math.toRadians(-122),
            endAngle5 = Math.toRadians(-165),
            endAngle6 = Math.toRadians(-150);

    public void mirrorAll() {
        control0 = control0.mirror();
        start = start.mirror();
        end0 = end0.mirror();
        end1 = end1.mirror();
        end2 = end2.mirror();
        end3 = end3.mirror();
        end4 = end4.mirror();
        end5 = end5.mirror();
        end6 = end6.mirror();
        end7 = end7.mirror();
        end8 = end8.mirror();
        end9 = end9.mirror();

        startAngle = mirrorAngleRad(startAngle);
        endAngle0 = mirrorAngleRad(endAngle0);
        endAngle1 = mirrorAngleRad(endAngle1);
        endAngle2 = mirrorAngleRad(endAngle2);
        endAngle3 = mirrorAngleRad(endAngle3);
        endAngle4 = mirrorAngleRad(endAngle4);
        endAngle5 = mirrorAngleRad(endAngle5);
        endAngle6 = mirrorAngleRad(endAngle6);

    }

    public double mirrorAngleRad(double angle) {
        return Math.PI - angle;
    }

    public void buildClose() {
        shootPreload = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierCurve(
                                start,
                                control0,
                                end0
                        )
                )
                .setLinearHeadingInterpolation(startAngle, endAngle0)
                .build();
        firstShoot = f.pathBuilder()
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
                .setReversed() .build();
        secondIntake = f.pathBuilder()
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
                .build();
        secondShoot = f.pathBuilder()
                .addPath(
                        // Path 6
                        new BezierLine(end5, end6)
                )
                .setTangentHeadingInterpolation()
                .setReversed().build();
        thirdShoot = f.pathBuilder()
                .addPath(
                        // Path 7
                        new BezierLine(end6, end7)
                )
                .setLinearHeadingInterpolation(endAngle4, endAngle5)
                .addPath(
                        // Path 8
                        new BezierLine(end7, end8)
                )
                .setLinearHeadingInterpolation(endAngle5, endAngle6)
                .addPath(
                        // Path 9
                        new BezierLine(end8, end9)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }


    public PathChain shootPreload;
    public PathChain firstShoot;
    public PathChain secondIntake;
    public PathChain secondShoot;
    public PathChain thirdShoot;
}
