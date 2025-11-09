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
            shoot = new Pose(51.0, 98.0),
            startIntake1 = new Pose(40.2, 88.3), // Intaking 1st
            endIntake1 = new Pose(17.7, 88.0),
            startIntake2 = new Pose(startIntake1.getX(), startIntake1.getY() - 24), // Intaking 2nd
            endIntake2 = new Pose(15.4, endIntake1.getY() - 24),
            startIntake3 = new Pose(startIntake1.getX(), startIntake2.getY() - 24), // Intaking 3rd
            endIntake3 = new Pose(15.4, endIntake2.getY() - 24);

    public static double
            startAngle = Math.toRadians(270),
            endAngle0 = Math.toRadians(-127),
            endAngle1 = Math.toRadians(-161),
            endAngle2 = Math.toRadians(240),
            endAngle3 = Math.toRadians(-157),
            endAngle4 = Math.toRadians(-122),
            startIntakeAngle = Math.toRadians(-155),
            endIntakeAngle = Math.toRadians(-150);

    public void mirrorAll() {
        control0 = control0.mirror();
        start = start.mirror();
        shoot = shoot.mirror();
        startIntake1 = startIntake1.mirror();
        endIntake1 = endIntake1.mirror();
        startIntake2 = startIntake2.mirror();
        endIntake2 = endIntake2.mirror();
        startIntake3 = startIntake3.mirror();
        endIntake3 = endIntake3.mirror();

        startAngle = mirrorAngleRad(startAngle);
        endAngle0 = mirrorAngleRad(endAngle0);
        endAngle1 = mirrorAngleRad(endAngle1);
        endAngle2 = mirrorAngleRad(endAngle2);
        endAngle3 = mirrorAngleRad(endAngle3);
        endAngle4 = mirrorAngleRad(endAngle4);
        startIntakeAngle = mirrorAngleRad(startIntakeAngle);
        endIntakeAngle = mirrorAngleRad(endIntakeAngle);

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
                                shoot
                        )
                )
                .setLinearHeadingInterpolation(startAngle, endAngle0)
                .build();
        firstShoot = f.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(shoot, startIntake1)
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Path 2
                        new BezierLine(startIntake1, endIntake1)
                )
                .setLinearHeadingInterpolation(endAngle0, endAngle1)
                .addPath(
                        // Path 3
                        new BezierLine(endIntake1, shoot)
                )
                .setTangentHeadingInterpolation()
                .setReversed() .build();
        secondIntake = f.pathBuilder()
                .addPath(
                        // Path 4
                        new BezierLine(shoot, startIntake2)
                )
                .setConstantHeadingInterpolation(endAngle2)
                .addPath(
                        // Path 5
                        new BezierLine(startIntake2, endIntake2)
                )
                .setLinearHeadingInterpolation(endAngle2, endAngle3)
                .build();
        secondShoot = f.pathBuilder()
                .addPath(
                        // Path 6
                        new BezierLine(endIntake2, shoot)
                )
                .setTangentHeadingInterpolation()
                .setReversed().build();
        thirdShoot = f.pathBuilder()
                .addPath(
                        // Path 7
                        new BezierLine(shoot, startIntake3)
                )
                .setLinearHeadingInterpolation(endAngle4, startIntakeAngle) // Intaking
                .addPath(
                        // Path 8
                        new BezierLine(startIntake3, endIntake3)
                )
                .setLinearHeadingInterpolation(startIntakeAngle, endIntakeAngle)
                .addPath(
                        // Path 9
                        new BezierLine(endIntake3, shoot)
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
