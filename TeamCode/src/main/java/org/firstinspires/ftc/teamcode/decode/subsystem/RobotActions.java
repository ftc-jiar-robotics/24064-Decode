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
            unstuckRecoveryTime = 0.1;

    public static Action setIntake(double power, double sleepSeconds) {
        return new ParallelAction(
                new InstantAction(() -> robot.intake.set(power, true)),
                new SleepAction(sleepSeconds),
                new InstantAction(() -> robot.shooter.setFeederIdle())
        );
    }

    public static Action armFlywheel() {
        return new InstantAction(() -> robot.shooter.setFlywheelManual(Flywheel.FlyWheelStates.ARMING));
    }

    public static Action shootArtifacts(int artifacts) {
        return new SequentialAction(
                new InstantAction(() -> robot.shooter.incrementQueuedShots(artifacts)),
                new InstantAction(() -> robot.intake.set(0.85)),
                new InstantAction(() -> unstuckTimer = System.nanoTime() / 1E9),
                telemetryPacket -> {
//                    if (robot.shooter.get() == Shooter.ShooterStates.RUNNING) {
//                        if (unstuckTimer + unstuckTime < System.nanoTime() / 1E9 && inUnstuckTimer) {
//                            robot.shooter.feeder.set(Feeder.FeederStates.OUTTAKING);
//                            inUnstuckTimer = false;
//                            unstuckRecoveryTimer = System.nanoTime() / 1E9;
//                        }
//
//                        if (!inUnstuckTimer && unstuckRecoveryTimer + unstuckRecoveryTime < System.nanoTime() / 1E9) {
//                            robot.shooter.feeder.set(Feeder.FeederStates.RUNNING);
//                            robot.intake.set(0.85);
//                            inUnstuckTimer = true;
//                            unstuckTimer = System.nanoTime() / 1E9;
//                        }
//                    }

                    return robot.shooter.getQueuedShots() != 0;

                },
                setIntake(0, 0)
            );
    }
}
