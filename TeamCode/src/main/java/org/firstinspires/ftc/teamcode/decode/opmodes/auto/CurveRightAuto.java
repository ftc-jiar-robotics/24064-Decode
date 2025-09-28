package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.CustomPaths;
import org.firstinspires.ftc.teamcode.decode.util.Actions;
import org.firstinspires.ftc.teamcode.decode.util.FollowPathAction;

@Autonomous(name = "Curve Right Auto")
public class CurveRightAuto extends AbstractAutoPedro {
    public CustomPaths C_P;
    private Follower f;

    @Override
    protected Pose getStartPose() {
        return new Pose(8.291, 80, Math.toRadians(0));
    }

    @Override
    protected void onInit() {
        f = robot.drivetrain;
        C_P = new CustomPaths(f);
        C_P.build();
    }

    @Override
    protected void onRun() {
        driveStraight();
        curveRight();
    }

    private void driveStraight() {
        C_P.straightLine.getPath(0).setTValueConstraint(.96);

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(0.8)), C_P.straightLine, 0.85, 0, f, "drive_straight_line"),
                                new FollowPathAction(f, C_P.straightLine)
                        )
                )
        );

        robot.actionScheduler.runBlocking();
    }

    private void curveRight() {
        C_P.curveRight.getPath(0).setTValueConstraint(.88);

        robot.actionScheduler.addAction(
                new SequentialAction(
                        new ParallelAction(
                                new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(0.6)), C_P.curveRight, 0.3, 0, f, "curve_right"),
                                new FollowPathAction(f, C_P.curveRight)
                        )
                )
        );

        robot.actionScheduler.runBlocking();
    }
}
