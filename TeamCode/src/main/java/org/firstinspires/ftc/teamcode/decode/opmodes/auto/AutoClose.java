package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.Paths;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.decode.util.Actions;
import org.firstinspires.ftc.teamcode.decode.util.FollowPathAction;

@Configurable
public class AutoClose extends AbstractAuto{
    private Follower f;
    private Paths path;

    @Override
    protected Pose getStartPose() {
        return Paths.start;
    }
    @Override
    protected void onInit() {
        f = robot.drivetrain;
        path = new Paths(f);
        path.buildClose();
    }
    @Override
    protected void onRun() {
        shootPreload();
        shootFirst();
        shootSecond();
        shootThird();
    }

    private void shootThird() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> f.setMaxPower(.3)),
                        new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.close, .1, 7, f, "speed_up_4"),
                        new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(.3)), path.close, .8, 7, f, "slow_down_3"),
                        new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.close, .01, 9, f, "speed_up_5"),
                        new Actions.CallbackAction(RobotActions.setIntake(1, 0), path.close, 0.8, 7, f, "start_intake_3"),
                        new Actions.CallbackAction(RobotActions.setIntake(0,0), path.close, 0.01, 9, f, "end_intake_3"),
                        new FollowPathAction(f, path.close.getPath(7)), // slows down to 30% so it turns slowly & shoots second 3 balls, then speeds up, then starts intake at 80% and slows back down to 30% to intake again
                        new FollowPathAction(f, path.close.getPath(8)), // intake balls
                        new FollowPathAction(f, path.close.getPath(9)), // speed up and end intake at 1% of path
                        RobotActions.shootArtifacts(3) // shoots third 3 balls
                )
        );

        robot.actionScheduler.runBlocking();
    }

    private void shootSecond() {
        robot.actionScheduler.addAction(
                new SequentialAction(
                        new InstantAction(() -> f.setMaxPower(.3)),
                        new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.close, .1, 4, f, "speed_up_2"),
                        new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(.3)), path.close, .8, 4, f, "slow_down_2"),
                        new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.close, .1, 6, f, "speed_up_3"),
                        new Actions.CallbackAction(RobotActions.setIntake(1, 0), path.close, 0.5, 4, f, "start_intake_2"),
                        new Actions.CallbackAction(RobotActions.setIntake(0,0), path.close, 0.98, 5, f, "end_intake_2"),
                        new FollowPathAction(f, path.close.getPath(4)), // slows down to 30% so it turns slowly & shoots first 3 balls, then speeds up, then starts intake at 50% and slows back down to 30% to intake again
                        new FollowPathAction(f, path.close.getPath(5)), // intake balls and the end intake at 98% of path so hatch opens
                        new SleepAction(0.5),
                        new FollowPathAction(f, path.close.getPath(6)), // speed up and go back to line
                        RobotActions.shootArtifacts(3) // shoots second 3 balls
                )
        );

        robot.actionScheduler.runBlocking();
    }

    private void shootFirst() {
        robot.actionScheduler.addAction(
             new SequentialAction(
                     new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(.3)), path.close, .8, 1, f, "slow_down_1"),
                     new Actions.CallbackAction(new InstantAction(() -> f.setMaxPower(1)), path.close, .01, 3, f, "speed_up_1"),
                     new Actions.CallbackAction(RobotActions.setIntake(1, 0), path.close, 0.5, 1, f, "start_intake_1"),
                     new Actions.CallbackAction(RobotActions.setIntake(0,0), path.close, 0.01, 3, f, "end_intake_1"),
                     new FollowPathAction(f, path.close.getPath(1)), // starts intake at 80% and start intake at 50% in path
                     new FollowPathAction(f, path.close.getPath(2)), // intake balls
                     new FollowPathAction(f, path.close.getPath(3)), // speed up and end intake at 1% of path
                     RobotActions.shootArtifacts(3) // shoots first 3 balls

             )
        );

        robot.actionScheduler.runBlocking();
    }

    private void shootPreload() {
        path.close.getPath(0).setTValueConstraint(0.88);

        robot.actionScheduler.addAction(
            new SequentialAction(
                    new FollowPathAction(f, path.close.getPath(0)),
                    RobotActions.shootArtifacts(3) // shoots preloaded 3 balls
            )
        );

        robot.actionScheduler.runBlocking();
    }
}
