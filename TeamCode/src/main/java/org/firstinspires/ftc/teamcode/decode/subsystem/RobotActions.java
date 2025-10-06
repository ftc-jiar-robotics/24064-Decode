package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

// TODO tune timings

public class RobotActions {
    private static double unstuckTimer = 0;
    private static double unstuckRecoveryTimer = 0;
    private static boolean inUnstuckTimer = true;

    public static double
            unstuckTime = 1.25,
            unstuckRecoveryTime = 0.5;

    public static Action setIntake(double power, double sleepSeconds) {
        return new ParallelAction(
                new InstantAction(() -> robot.intake.set(power, true)),
                new SleepAction(sleepSeconds)
        );
    }

    public static Action setShooter(Shooter.ShooterStates targetState, double sleepSeconds) {
        return new ParallelAction(
                new InstantAction(() -> robot.shooter.set(targetState, true)),
                new SleepAction(sleepSeconds)
        );
    }

    private static Action runManual(double power, double sleepSeconds) {
        return new ParallelAction(
                new InstantAction(() -> robot.shooter.runManual(power)),
                new SleepAction(sleepSeconds)
        );
    }


    public static Action shootArtifacts(int artifacts) {
        return new SequentialAction(
                new InstantAction(() -> robot.shooter.incrementQueuedShots(artifacts)),
                setIntake(1, 0),
                new InstantAction(() -> unstuckTimer = System.nanoTime() / 1E9),
                telemetryPacket -> {
                    if (robot.shooter.get() == Shooter.ShooterStates.RUNNING) {
                        if (unstuckTimer + unstuckTime < System.nanoTime() / 1E9 && inUnstuckTimer) {
                            robot.shooter.feeder.set(Feeder.FeederStates.OUTTAKING);
                            robot.intake.set(-0.5);
                            inUnstuckTimer = false;
                            unstuckRecoveryTimer = System.nanoTime() / 1E9;
                        }

                        if (!inUnstuckTimer && unstuckRecoveryTimer + unstuckRecoveryTime < System.nanoTime() / 1E9) {
                            robot.shooter.feeder.set(Feeder.FeederStates.RUNNING);
                            robot.intake.set(1.0);
                            inUnstuckTimer = true;
                            unstuckTimer = System.nanoTime() / 1E9;
                        }
                    }

                    return robot.shooter.getQueuedShots() != 0;

                },
                setIntake(0, 0)
            );
    }

    public static Action decrementQueuedShots() {
        return new InstantAction(() -> robot.shooter.decrementQueuedShots());
    }

    // TODO finish manual action
    public static Action shootArtifactManually(double power) {
        return new SequentialAction(
                setShooter(Shooter.ShooterStates.MANUAL, 0.2),
                runManual(power, 0.4),
                setShooter(Shooter.ShooterStates.RUNNING, 0)
        );
    }
}
