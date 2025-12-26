package org.firstinspires.ftc.teamcode.decode.opmodes.auto.path;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;

public class ConfigPaths {
    private final Follower f;
    public ConfigPaths(Follower follower) {
        f = follower;
    }

    public static boolean isPathRed = false;

    public static long MAX_HP_TIME_MS = 567;
    public static double LEAVE_TIME = 29.5;

    public static Pose
            controlShootClose = new Pose(47.6, 113.1),
            controlIntakeHP = new Pose(57.3, 17.4),
            controlGate = new Pose(27.1, 72.4);

    public static Pose
            startFar = Common.BLUE_SMALL_TRIANGLE,
            startClose = Common.BLUE_BIG_TRIANGLE,
            shootClose = new Pose(51.0, 101.0),
            shootFar = new Pose(55.5, 11.3),
            intakeHPEnd = new Pose(3.5, 8.15),
            intakeHPMid = new Pose(6.5, 8.15),
            intakeFirstStart = new Pose(38.2, 84.5),
            intakeFirstEnd = new Pose(17.1, 84.5),
            intakeSecondStart = new Pose(38.2, 60.0),
            intakeSecondEnd = new Pose(17.1, 60.0),
            intakeThirdStart = new Pose(38.2, 35.5),
            intakeThirdEnd = new Pose(17.1, 35.5),
            gateOpen = new Pose(12.0, 67.0),
            gateIntake = new Pose(9.0, 59.5),
            leaveClose = new Pose(37.5,90.1),
            leaveFar = new Pose(49.6, 16.2);


    public static double
            startAngleClose = Math.toRadians(270),
            startAngleFar = Math.toRadians(90),
            shootAngleFar = Math.toRadians(167),
            shootAngleClose = Math.toRadians(-127),
            intakeAngle = Math.toRadians(180),
            gateIntakeAngle = Math.toRadians(165);

    public double mirrorAngleRad(double angle) {
        return Math.PI - angle;
    }

    public void mirrorAll() {
        startFar = startFar.mirror();
        startClose = startClose.mirror();
        shootFar = shootFar.mirror();
        shootClose = shootClose.mirror();
        intakeHPEnd = intakeHPEnd.mirror();
        intakeHPMid = intakeHPMid.mirror();
        intakeFirstStart = intakeFirstStart.mirror();
        intakeFirstEnd = intakeFirstEnd.mirror();
        intakeSecondStart = intakeSecondStart.mirror();
        intakeSecondEnd = intakeSecondEnd.mirror();
        intakeThirdStart = intakeThirdStart.mirror();
        intakeThirdEnd = intakeThirdEnd.mirror();
        gateOpen = gateOpen.mirror();
        gateIntake = gateIntake.mirror();
        leaveClose = leaveClose.mirror();
        leaveFar = leaveFar.mirror();

        controlShootClose = controlShootClose.mirror();
        controlIntakeHP = controlIntakeHP.mirror();
        controlGate = controlGate.mirror();

        startAngleClose = mirrorAngleRad(startAngleClose);
        startAngleFar = mirrorAngleRad(startAngleFar);
        shootAngleFar = mirrorAngleRad(shootAngleFar);
        shootAngleClose = mirrorAngleRad(shootAngleClose);
        intakeAngle = mirrorAngleRad(intakeAngle);
        gateIntakeAngle = mirrorAngleRad(gateIntakeAngle);
    }

    public void configAutoBuild(boolean isBigTriangle) {
        if (isBigTriangle) {
            preload = f.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    startClose,
                                    controlShootClose,
                                    shootClose
                            )
                    )
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(f::getHeading, shootAngleClose, 0.45))
                    .build();
        } else {
            preload = f.pathBuilder()
                    .addPath(
                            new BezierLine(startFar, shootFar)
                    )
                    .setHeadingInterpolation(HeadingInterpolator.facingPoint(shootFar))
                    .build();
        }

        intakeHP = f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                f::getPose,
                                controlIntakeHP,
                                intakeHPEnd
                        )
                )
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(f::getHeading, intakeAngle, 0.65))
                .build();

        intakeHPBack = f.pathBuilder()
                .addPath(new BezierLine(f::getPose, intakeHPMid))
                .setConstantHeadingInterpolation(intakeAngle)
                .build();

        intakeHPRetry = f.pathBuilder()
                .addPath(new BezierLine(intakeHPMid, intakeHPEnd))
                .setConstantHeadingInterpolation(intakeAngle)
                .build();

        intakeFirst = f.pathBuilder()
                .addPath(
                        new BezierCurve(f::getPose, intakeFirstStart)
                )
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(f::getHeading, intakeAngle, 0.8))
                .addPath(
                        new BezierLine(intakeFirstStart, intakeFirstEnd)
                )
                .setConstantHeadingInterpolation(intakeAngle)
                .build();

        intakeSecond = f.pathBuilder()
                .addPath(
                        new BezierLine(f::getPose, intakeSecondStart)
                )
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(f::getHeading, intakeAngle, 0.8))
                .addPath(
                        new BezierLine(intakeSecondStart, intakeSecondEnd)
                )
                .setConstantHeadingInterpolation(intakeAngle)
                .build();

        intakeThird = f.pathBuilder()
                .addPath(
                        new BezierCurve(f::getPose, intakeThirdStart)
                )
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(f::getHeading, intakeAngle, 0.8))
                .addPath(
                        new BezierLine(intakeThirdStart, intakeThirdEnd)
                )
                .setConstantHeadingInterpolation(intakeAngle)
                .build();

        intakeGate = f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                f::getPose,
                                controlGate,
                                gateOpen
                        )
                )
                .addPath(new BezierLine(gateOpen, gateIntake))
                .setGlobalHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(
                                0,
                                0.6,
                                HeadingInterpolator.linearFromPoint(f::getHeading, intakeAngle, 0.7)
                        ),
                        new HeadingInterpolator.PiecewiseNode(
                                0.6,
                                1,
                                HeadingInterpolator.facingPoint(gateOpen)
                        )
                ))
                .build();

        intakeGateBack = f.pathBuilder()
                .addPath(new BezierLine(gateIntake, gateOpen))
                .setLinearHeadingInterpolation(gateIntakeAngle, intakeAngle)
                .build();

        openGate = f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                f::getPose,
                                controlGate,
                                gateOpen
                        )
                )
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(f::getHeading, intakeAngle, 0.8))
                .build();

        if (isBigTriangle) {
            shoot = f.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    f::getPose,
                                    controlShootClose,
                                    shootClose
                            )
                    )
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(f::getHeading, shootAngleClose, 0.45))
                    .build();
        } else {
            shoot = f.pathBuilder()
                    .addPath(
                            new BezierLine(f::getPose, shootFar)
                    )
                    .setHeadingInterpolation(HeadingInterpolator.facingPoint(shootFar))
                    .build();
        }

        leave = f.pathBuilder()
                .addPath(
                        new BezierLine(f::getPose, isBigTriangle ? leaveClose : leaveFar)
                )
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(f::getHeading, isBigTriangle ? startAngleClose : startAngleFar, 0.8))
                .build();
    }

    public PathChain preload;
    public PathChain intakeHP;
    public PathChain intakeHPBack;
    public PathChain intakeHPRetry;
    public PathChain intakeFirst;
    public PathChain intakeSecond;
    public PathChain intakeThird;
    public PathChain intakeGate;
    public PathChain intakeGateBack;
    public PathChain openGate;
    public PathChain shoot;
    public PathChain leave;

}
