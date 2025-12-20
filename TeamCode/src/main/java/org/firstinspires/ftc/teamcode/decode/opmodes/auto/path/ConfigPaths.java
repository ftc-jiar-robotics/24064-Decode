package org.firstinspires.ftc.teamcode.decode.opmodes.auto.path;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class ConfigPaths {
    private final Follower f;
    public ConfigPaths(Follower follower) {
        f = follower;
    }

    public static Pose
            controlPreload = new Pose(47.6, 113.1),
            controlIntakeHP = new Pose(32.5, 10),
            controlIntakeFirst = new Pose(65.0, 85.0),
            controlIntakeSecond = new Pose(65.0, 60.5),
            controlIntakeThird = new Pose(65.0, 36.0),
            controlGate = new Pose(34.5, 63);

    public static Pose
            startFar = new Pose(55.5, 7.25, Math.toRadians(90)),
            startClose = new Pose(30.5, 135.5, Math.toRadians(270)),
            shootClose = new Pose(51.0, 101.0),
            shootFar = new Pose(55.5, 9.3),
            intakeHPEnd = new Pose(3.5, 8.15),
            intakeFirstStart = new Pose(34.8, 84.5),
            intakeFirstEnd = new Pose(11.8, 84.5),
            intakeSecondStart = new Pose(34.8, 60.0),
            intakeSecondEnd = new Pose(11.8, 60.0),
            intakeThirdStart = new Pose(34.8, 35.5),
            intakeThirdEnd = new Pose(11.8, 35.5),
            gateIntake = new Pose(10.0, 67.0);

    public static double
            startAngleClose = Math.toRadians(270),
            startAngleFar = Math.toRadians(90),
            shootAngle = Math.toRadians(-127),
            intakeAngle = Math.toRadians(180),
            gateIntakeAngle = Math.toRadians(165);

    public void configAutoBuild(boolean isAudienceSide) {
        if (isAudienceSide) {
            preload = f.pathBuilder()
                    .addPath(
                            // Path 0
                            new BezierLine(startFar, shootFar)
                    )
                    .setConstantHeadingInterpolation(startAngleFar)
                    .build();
        } else {
            preload = f.pathBuilder()
                    .addPath(
                            // Path 0
                            new BezierCurve(
                                    startClose,
                                    controlPreload,
                                    shootClose
                            )
                    )
                    .setLinearHeadingInterpolation(startAngleClose, shootAngle)
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
                .setTangentHeadingInterpolation()
                .build();

        intakeFirst = f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                f::getPose,
                                controlIntakeFirst,
                                intakeFirstStart
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(intakeFirstStart, intakeFirstEnd)
                )
                .setConstantHeadingInterpolation(intakeAngle)
                .build();

        intakeSecond = f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                f::getPose,
                                controlIntakeSecond,
                                intakeSecondStart
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(intakeSecondStart, intakeSecondEnd)
                )
                .setConstantHeadingInterpolation(intakeAngle)
                .build();

        intakeThird = f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                f::getPose,
                                controlIntakeThird,
                                intakeThirdStart
                        )
                )
                .setTangentHeadingInterpolation()
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
                                gateIntake
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), gateIntakeAngle)
                .build();
    }

    public PathChain preload;
    public PathChain intakeHP;
    public PathChain intakeFirst;
    public PathChain intakeSecond;
    public PathChain intakeThird;
    public PathChain intakeGate;
    public PathChain openGate;
    public PathChain shoot;

}
