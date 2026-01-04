package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.SLOW_MODE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isSlowMode;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.decode.util.Actions;

// TODO tune timings

public class RobotActions {
    private static final ElapsedTime shotTimer = new ElapsedTime();

    private static boolean doSetSlowMode = false;

    public static Action setIntake(double power, double sleepSeconds) {
        return new ParallelAction(
                new InstantAction(() -> robot.intake.set(power, true)),
                new SleepAction(sleepSeconds),
                new InstantAction(() -> robot.shooter.setFeederIdle(Math.abs(power) > 0))
        );
    }

    private static boolean getDoSetSlowMode() {
        return doSetSlowMode;
    }

    public static Action shootArtifacts(int artifacts) {
        return shootArtifacts(artifacts, Double.POSITIVE_INFINITY, true);
    }

    public static Action shootArtifacts(int artifacts, double seconds) {
        return shootArtifacts(artifacts, seconds, true);
    }

    public static Action shootArtifacts(int artifacts, double seconds, boolean setSlowMode) {
        return new Actions.SingleCheckAction(
                robot.shooter::isBallPresent,
                new SequentialAction(
                        new InstantAction(() -> doSetSlowMode = setSlowMode),
                        new InstantAction(() -> robot.shooter.incrementQueuedShots(artifacts)),
                        new InstantAction(() -> robot.intake.set(1.0)),
                        new Actions.SingleCheckAction(
                                RobotActions::getDoSetSlowMode,
                                new InstantAction(() -> {
                                    robot.drivetrain.setMaxPowerScaling(SLOW_MODE);
                                    isSlowMode = true;
                                })
                        ),
                        new InstantAction(shotTimer::reset),
                        telemetryPacket -> robot.shooter.getQueuedShots() > 0 && shotTimer.seconds() <= seconds,
                        new InstantAction(() -> robot.shooter.clearQueueShots()),
                        setIntake(0, 0)
//                        new Actions.SingleCheckAction(
//                                RobotActions::getDoSetSlowMode,
//                                new InstantAction(() -> {
//                                    robot.drivetrain.setMaxPowerScaling(1);
//                                    isSlowMode = false;
//                                })
//                        )
                )
        );
    }

    public static Action emergencyShootArtifacts() {
        return new SequentialAction(
                new InstantAction(() -> robot.shooter.turnOnEmergency()),
                new InstantAction(() -> robot.intake.set(1.0)),
                new InstantAction(() -> {
                    robot.drivetrain.setMaxPowerScaling(SLOW_MODE);
                    isSlowMode = true;
                }),
                new InstantAction(shotTimer::reset),
                telemetryPacket -> robot.shooter.getQueuedShots() > 0,
                new InstantAction(() -> robot.shooter.clearQueueShots()),
                setIntake(0, 0),
                new InstantAction(() -> {
                    robot.drivetrain.setMaxPowerScaling(1);
                    isSlowMode = false;
                })
        );
    }

    public static Action armFlywheel() {
        return new InstantAction(() -> robot.shooter.setFlywheelManual(Flywheel.FlyWheelStates.ARMING));
    }

    public static Action armTurret() {
        return new InstantAction(() -> robot.shooter.setTurretManual(Turret.TurretStates.ODOM_TRACKING));
    }
}
