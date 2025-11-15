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
            control1 = new Pose(22.1, 78.4),
            start = new Pose(30.5, 135.5, Math.toRadians(270)),
            shoot = new Pose(51.0, 98.0),
            gate = new Pose(15.5, 71.6),
            leave = new Pose(37.5,90.1),
            gateCycleCP = new Pose(44.000, 65.700),
            gateCycleOpen = new Pose(11.000, 61.900),
            gateCycleIntake = new Pose(11.000, 55.700),
            startIntake1 = new Pose(40.2, 88.3), // Intaking 1st
            endIntake1 = new Pose(17.7, 88.0),
            startIntake2 = new Pose(startIntake1.getX(), startIntake1.getY() - 24), // Intaking 2nd
            endIntake2 = new Pose(15.4, endIntake1.getY() - 24),
            startIntake3 = new Pose(startIntake1.getX(), startIntake2.getY() - 24), // Intaking 3rd
            endIntake3 = new Pose(15.4, endIntake2.getY() - 24);

    public static double
            gateAngle = Math.toRadians(-180),
            startAngle = Math.toRadians(270),
            shootAngle = Math.toRadians(-127),
            gateCycleShootAngle = Math.toRadians(250),
            gateCycleIntakeAngle = Math.toRadians(140),
            startIntakeAngle = Math.toRadians(-155),
            endIntakeAngle = Math.toRadians(-150);

    public void mirrorAll() {
        control0 = control0.mirror();
        control1 = control1.mirror();
        gate = gate.mirror();
        start = start.mirror();
        shoot = shoot.mirror();
        leave = leave.mirror();
        startIntake1 = startIntake1.mirror();
        endIntake1 = endIntake1.mirror();
        startIntake2 = startIntake2.mirror();
        endIntake2 = endIntake2.mirror();
        startIntake3 = startIntake3.mirror();
        endIntake3 = endIntake3.mirror();
        gateCycleCP = gateCycleCP.mirror();
        gateCycleOpen = gateCycleOpen.mirror();
        gateCycleIntake = gateCycleIntake.mirror();

        startAngle = mirrorAngleRad(startAngle);
        shootAngle = mirrorAngleRad(shootAngle);
        startIntakeAngle = mirrorAngleRad(startIntakeAngle);
        gateAngle = mirrorAngleRad(gateAngle);
        endIntakeAngle = mirrorAngleRad(endIntakeAngle);
        gateCycleIntakeAngle = mirrorAngleRad(gateCycleIntakeAngle);
        gateCycleShootAngle = mirrorAngleRad(gateCycleShootAngle);

    }

    public double mirrorAngleRad(double angle) {
        return Math.PI - angle;
    }

    public void goal12Build() {
        shootPreload = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierCurve(
                                start,
                                control0,
                                shoot
                        )
                )
                .setLinearHeadingInterpolation(startAngle, shootAngle)
                .build();
        firstIntake = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(shoot, startIntake1)
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Path 1
                        new BezierLine(startIntake1, endIntake1)
                )
                .setLinearHeadingInterpolation(startIntakeAngle, endIntakeAngle)
                .addPath( // Gate
                        // Path 2
                        new BezierCurve(
                                endIntake1,
                                control1,
                                gate
                        )
                )
                .setConstantHeadingInterpolation(gateAngle)
                .setReversed() .build();
        firstShoot = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(gate, shoot)
                )
                .setConstantHeadingInterpolation(gateAngle)
                .setReversed() .build();
        secondIntake = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(shoot, startIntake2)
                )
                .setConstantHeadingInterpolation(startIntakeAngle)
                .addPath(
                        // Path 1
                        new BezierLine(startIntake2, endIntake2)
                )
                .setLinearHeadingInterpolation(startIntakeAngle, endIntakeAngle)
                .addPath(
                        // Path 2
                        new BezierLine(endIntake2, shoot)
                )
                .setTangentHeadingInterpolation()
                .setReversed().build();
        thirdShoot = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(shoot, startIntake3)
                )
                .setLinearHeadingInterpolation(shootAngle, startIntakeAngle) // Intaking
                .addPath(
                        // Path 1
                        new BezierLine(startIntake3, endIntake3)
                )
                .setLinearHeadingInterpolation(startIntakeAngle, endIntakeAngle)
                .addPath(
                        // Path 2
                        new BezierLine(endIntake3, shoot)
                )
                .setTangentHeadingInterpolation()
                .setReversed().build();
        goalLeave = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(shoot, leave)
                )
                .setTangentHeadingInterpolation()
                .setReversed().build();
    }

    public void goal18Build() {
        shootPreload = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierCurve(
                                start,
                                control0,
                                shoot
                        )
                )
                .setLinearHeadingInterpolation(startAngle, shootAngle)
                .build();

        secondIntake = f.pathBuilder()
                .addPath(
                        // Path 4
                        new BezierLine(shoot, startIntake2)
                )
                .setConstantHeadingInterpolation(startIntakeAngle)
                .addPath(
                        // Path 5
                        new BezierLine(startIntake2, endIntake2)
                )
                .setLinearHeadingInterpolation(startIntakeAngle, gateAngle)
                .build();
        secondShoot = f.pathBuilder()
                .addPath(
                        // Path 6
                        new BezierLine(endIntake2, shoot)
                )
                .setTangentHeadingInterpolation()
                .setReversed().build();
        cycleGate = f.pathBuilder()
                .addPath(
                        // cycleStart
                        new BezierCurve(
                                shoot,
                                gateCycleCP,
                                gateCycleOpen
                        )
                )
                .setLinearHeadingInterpolation(gateCycleShootAngle, gateCycleIntakeAngle)
                .addPath(
                        // Path 3
                        new BezierLine(gateCycleOpen, gateCycleIntake)
                )
                .setConstantHeadingInterpolation(gateCycleIntakeAngle)
                .addPath(
                        // cycleEnd
                        new BezierCurve(
                                gateCycleIntake,
                                gateCycleCP,
                                shoot
                        )
                )
                .setLinearHeadingInterpolation(gateCycleIntakeAngle, gateCycleShootAngle)
                .build();
    }


    public PathChain shootPreload;
    public PathChain firstIntake;
    public PathChain firstShoot;
    public PathChain secondIntake;
    public PathChain secondShoot;
    public PathChain thirdShoot;
    public PathChain goalLeave;
    public PathChain cycleGate;
}
