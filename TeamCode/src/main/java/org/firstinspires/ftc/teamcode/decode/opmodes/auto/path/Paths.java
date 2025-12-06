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

    //TODO all HP angles + positions need to be tuned

    public static Pose
            control0 = new Pose(47.6, 113.1),
            gateControl12 = new Pose(22.1, 78.4),
            gateControl15 = new Pose(27.1, 72.4),
            start = new Pose(30.5, 135.5, Math.toRadians(270)),
            shoot = new Pose(51.0, 101.0),
            gate = new Pose(15.5, 73),
            leave = new Pose(37.5,90.1),
            gateCycleCP = new Pose(30.000, 55.700),
            gateCycleOpen = new Pose(12.200, 64.800),
            gateCycleIntake = new Pose(11.900, 64.60),
            startIntake1 = new Pose(38.2, 91.3), // Intaking 1st
            endIntake1 = new Pose(17.7, 91.0),
            startIntake2 = new Pose(startIntake1.getX(), startIntake1.getY() - 24), // Intaking 2nd
            endIntake2 = new Pose(17.7, endIntake1.getY() - 24),
            startIntake3 = new Pose(startIntake1.getX(), startIntake2.getY() - 24), // Intaking 3rd
            endIntake3 = new Pose(17.7, endIntake2.getY() - 24),
            endIntakeHP = new Pose(9.300, 10.600),
            midIntakeHP = new Pose(18.300, 10.600),
            controlHP = new Pose(57.300, 17.400);

    public static double
            gateAngle = Math.toRadians(180),
            startAngle = Math.toRadians(270),
            shootAngle = Math.toRadians(-127),
            gateCycleShootAngle = Math.toRadians(215),
            gateCycleIntakeAngle = Math.toRadians(135),
            startIntakeAngle = Math.toRadians(-155),
            endIntakeAngle = Math.toRadians(-150),
            startIntakeAngleHP = Math.toRadians(180),
            shootHPAngle = Math.toRadians(-114);


    public void mirrorAll() {
        control0 = control0.mirror();
        gateControl12 = gateControl12.mirror();
        gateControl15 = gateControl15.mirror();
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
        endIntakeHP = endIntakeHP.mirror();
        midIntakeHP = midIntakeHP.mirror();
        gateCycleCP = gateCycleCP.mirror();
        gateCycleOpen = gateCycleOpen.mirror();
        gateCycleIntake = gateCycleIntake.mirror();
        controlHP = controlHP.mirror();

        startAngle = mirrorAngleRad(startAngle);
        shootAngle = mirrorAngleRad(shootAngle);
        startIntakeAngle = mirrorAngleRad(startIntakeAngle);
        gateAngle = mirrorAngleRad(gateAngle);
        endIntakeAngle = mirrorAngleRad(endIntakeAngle);
        startIntakeAngleHP = mirrorAngleRad(startIntakeAngleHP);
        gateCycleIntakeAngle = mirrorAngleRad(gateCycleIntakeAngle);
        gateCycleShootAngle = mirrorAngleRad(gateCycleShootAngle);
        shootHPAngle = mirrorAngleRad(shootHPAngle);

    }

    public double mirrorAngleRad(double angle) {
        return Math.PI - angle;
    }


    public void goal15Build(){
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
                .addPath(
                        // Path 2
                        new BezierLine(endIntake1, shoot)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

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
                .addPath( // Gate
                        // Path 2
                        new BezierCurve(
                                endIntake2,
                                gateControl15,
                                gate
                        )
                )
                .setConstantHeadingInterpolation(gateAngle)
                .build();

        secondShoot = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(gate, shoot)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

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
        humanPlayerIntake0 = f.pathBuilder()
                .addPath(
                    // Path 0
                    new BezierCurve(
                            shoot,
                            controlHP,
                            endIntakeHP
                    )
                )
                .setTangentHeadingInterpolation()
                .build();

        humanPlayerIntake1 = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(endIntakeHP, midIntakeHP)
                )
                .setConstantHeadingInterpolation(startIntakeAngleHP)
                .build();

        humanPlayerShoot = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(midIntakeHP, endIntakeHP)
                )
                .setConstantHeadingInterpolation(startIntakeAngleHP)
                .addPath(
                        // Path 1
                        new BezierLine(endIntakeHP, shoot)
                )
                .setConstantHeadingInterpolation(shootHPAngle)
                .build();

        goalLeave = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(shoot, leave)
                )
                .setTangentHeadingInterpolation()
                .build();
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
                                gateControl12,
                                gate
                        )
                )
                .setConstantHeadingInterpolation(gateAngle)
                .build();

        firstShoot = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(gate, shoot)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

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
                .setReversed()
                .build();

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
                .build();
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
                .addPath(
                        // Path 2
                        new BezierLine(endIntake1, shoot)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

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
                .addPath( // Gate
                        // Path 2
                        new BezierCurve(
                                endIntake2,
                                gateControl15,
                                gate
                        )
                )
                .setConstantHeadingInterpolation(gateAngle)
                .build();

        secondShoot = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(gate, shoot)
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeGate = f.pathBuilder()
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
                .build();

        shootGate = f.pathBuilder()
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

        goalLeave = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(shoot, leave)
                )
                .setTangentHeadingInterpolation()
                .build();
    }


    public PathChain shootPreload;
    public PathChain firstIntake;
    public PathChain firstShoot;
    public PathChain secondIntake;
    public PathChain secondShoot;
    public PathChain thirdIntake;
    public PathChain thirdShoot;
    public PathChain humanPlayerIntake0;
    public PathChain humanPlayerIntake1;
    public PathChain humanPlayerShoot;
    public PathChain goalLeave;
    public PathChain intakeGate;
    public PathChain shootGate;
}
