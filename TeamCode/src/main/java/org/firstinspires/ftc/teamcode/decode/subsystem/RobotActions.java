package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.SLOW_MODE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isSlowMode;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

// TODO tune timings

public class RobotActions {
    public static Action setIntake(double power, double sleepSeconds) {
        return new ParallelAction(
                new InstantAction(() -> robot.intake.set(power, true)),
                new SleepAction(sleepSeconds),
                new InstantAction(() -> robot.shooter.setFeederIdle(Math.abs(power) > 0))
        );
    }

    public static Action shootArtifacts(int artifacts) {
        return new SequentialAction(
                new InstantAction(() -> robot.shooter.incrementQueuedShots(artifacts)),
                new InstantAction(() -> robot.intake.set(0.85)),
                new InstantAction(() -> {
                    robot.drivetrain.setMaxPowerScaling(SLOW_MODE);
                    isSlowMode = true;
                }),
                telemetryPacket -> robot.shooter.getQueuedShots() != 0,
                setIntake(0, 0),
                new InstantAction(() -> {
                    robot.drivetrain.setMaxPowerScaling(1);
                    isSlowMode = false;
                })
            );
    }
}
