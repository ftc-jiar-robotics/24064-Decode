package org.firstinspires.ftc.teamcode.decode.opmodes.auto.path;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

@Configurable
public class GoalPaths {
    private final Follower f;
    public GoalPaths(Follower follower) {
        f = follower;
    }

    public static boolean isPathRed = false;
    public static long
            MAX_HP_TIME_MS = 670,
            MAX_HP_GOING_MS = 6000;
    public static double LEAVE_TIME = 29;

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
            controlHP = new Pose(57.300, 17.400),
            preload21 = new Pose(49.800, 93.600),
            shoot21 = new Pose(61.500, 83.700),
            intake21Control = new Pose(60.800, 60.100),
            endIntakeSecond21 = new Pose(8.400, 59.100),
            gateCycleControl21 = new Pose(15.9, 56.900),
            intakeGateCycle21 = new Pose(12.0, 63.000),
            endIntakeFirst21 = new Pose(20.0, 83.900),
            intakeThirdControl21 = new Pose(59.700, 35.900),
            startIntakeThird21 = new Pose(46.700, 35.900),
            endIntakeThird21 = new Pose(14.400, 36.200),
            leave21 = new Pose(53.8, 74.7);



    public static double
            gateAngle = Math.toRadians(180),
            startAngle = Math.toRadians(270),
            shootAngle = Math.toRadians(-127),
            gateCycleShootAngle = Math.toRadians(215),
            gateCycleIntakeAngle = Math.toRadians(160),
            startIntakeAngle = Math.toRadians(-155),
            endIntakeAngle = Math.toRadians(-150),
            startIntakeAngleHP = Math.toRadians(180),
            shootHPAngle = Math.toRadians(-114),
            intake21Angle = Math.toRadians(180),
            goHome21Angle = Math.toRadians(-139);



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
        preload21 = preload21.mirror();
        shoot21 = shoot21.mirror();
        intake21Control = intake21Control.mirror();
        endIntakeSecond21 = endIntakeSecond21.mirror();
        gateCycleControl21 = gateCycleControl21.mirror();
        intakeGateCycle21 = intakeGateCycle21.mirror();
        endIntakeFirst21 = endIntakeFirst21.mirror();
        intakeThirdControl21 = intakeThirdControl21.mirror();
        startIntakeThird21 = startIntakeThird21.mirror();
        endIntakeThird21 = endIntakeThird21.mirror();
        leave21 = leave21.mirror();

        startAngle = mirrorAngleRad(startAngle);
        shootAngle = mirrorAngleRad(shootAngle);
        startIntakeAngle = mirrorAngleRad(startIntakeAngle);
        gateAngle = mirrorAngleRad(gateAngle);
        endIntakeAngle = mirrorAngleRad(endIntakeAngle);
        startIntakeAngleHP = mirrorAngleRad(startIntakeAngleHP);
        gateCycleIntakeAngle = mirrorAngleRad(gateCycleIntakeAngle);
        gateCycleShootAngle = mirrorAngleRad(gateCycleShootAngle);
        shootHPAngle = mirrorAngleRad(shootHPAngle);
        intake21Angle = mirrorAngleRad(intake21Angle);
        goHome21Angle = mirrorAngleRad(goHome21Angle);

    }

    public double mirrorAngleRad(double angle) {
        return Math.PI - angle;
    }



    public void goal21Build(){
        shootPreload21 = f.pathBuilder()
                .addPath(
                        new BezierLine(start, preload21)
                )
                .setLinearHeadingInterpolation(startAngle, intake21Angle)
                .addPath(
                        new BezierLine(preload21, shoot21)
                )
                .setConstantHeadingInterpolation(intake21Angle)
                .build();
        secondIntake21 = f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shoot21,
                                intake21Control,
                                endIntakeSecond21
                        )
                )
                .setConstantHeadingInterpolation(intake21Angle)
                .addPath(
                        new BezierCurve(
                                endIntakeSecond21,
                                intake21Control,
                                shoot21
                        )
                )
                .setConstantHeadingInterpolation(intake21Angle)
                .build();
        gateCycleIntake21 = f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shoot21,
                                gateCycleControl21,
                                intakeGateCycle21
                        )
                )
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                .2,
                                HeadingInterpolator.tangent
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                0.2,
                                1,
                                HeadingInterpolator.constant(gateCycleIntakeAngle)
                        )
                ))
                .build();
        gateCycleShoot21 = f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                intakeGateCycle21,
                                gateCycleControl21,
                                shoot21
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed().build();
        firstIntake21 = f.pathBuilder()
                .addPath(
                        new BezierLine(shoot21, endIntakeFirst21)
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(endIntakeFirst21, shoot21)
                )
                .setTangentHeadingInterpolation().setReversed().build();
        thirdIntake21 = f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shoot21,
                                intakeThirdControl21,
                                startIntakeThird21
                        )
                )
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                .4,
                                HeadingInterpolator.tangent
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                        0.4,
                        1,
                        HeadingInterpolator.constant(intake21Angle)
                        )
                ))
                .addPath(
                        new BezierLine(startIntakeThird21, endIntakeThird21)
                )
                .setConstantHeadingInterpolation(intake21Angle)
                .addPath(
                        new BezierLine(endIntakeThird21, shoot21)
                )
                .setConstantHeadingInterpolation(goHome21Angle).build();
        goalLeave21 = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(shoot21, leave21)
                )
                .setTangentHeadingInterpolation()
                .build();

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
                        new BezierLine(f::getPose, midIntakeHP)
                )
                .setConstantHeadingInterpolation(startIntakeAngleHP)
                .build();
        humanPlayerIntake1_5 = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(f::getPose, endIntakeHP)
                )
                .setConstantHeadingInterpolation(startIntakeAngleHP)
                .build();

        humanPlayerShoot = f.pathBuilder()
                .addPath(
                        // Path 0
                        new BezierLine(f::getPose, shoot)
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


    public PathChain shootPreload;
    public PathChain firstIntake;
    public PathChain firstShoot;
    public PathChain secondIntake;
    public PathChain secondShoot;
    public PathChain thirdShoot;
    public PathChain humanPlayerIntake0;
    public PathChain humanPlayerIntake1;
    public PathChain humanPlayerIntake1_5;
    public PathChain humanPlayerShoot;
    public PathChain goalLeave;
    public PathChain shootPreload21;
    public PathChain firstIntake21;
    public PathChain secondIntake21;
    public PathChain thirdIntake21;
    public PathChain gateCycleIntake21;
    public PathChain gateCycleShoot21;
    public PathChain goalLeave21;
}
