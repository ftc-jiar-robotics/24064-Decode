package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isHoodManual;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Shooter extends Subsystem<Shooter.ShooterStates> {
    final Hood hood;
    final Flywheel flywheel;
    final Turret turret;
    final Feeder feeder;

    private boolean didCurrentDrop;


    private int queuedShots = 0;

    public enum ShooterStates {
        IDLE, PREPPING, RUNNING
    }

    private ShooterStates targetState = ShooterStates.IDLE;

    public Shooter(HardwareMap hw) {
        this.hood = new Hood(hw);
        this.flywheel = new Flywheel(hw);
        this.turret = new Turret(hw);
        this.feeder = new Feeder(hw);
    }

    @Override
    public void set(ShooterStates t) {
        targetState = t;
    }

    @Override
    public ShooterStates get() {
        return targetState;
    }

    @Override
    public void setLocked(boolean isLocked) {
        super.setLocked(isLocked);
        feeder.setLocked(isLocked);
        flywheel.setLocked(isLocked);
        turret.setLocked(isLocked);
        hood.setLocked(isLocked);
    }

    public int getQueuedShots() {
        return queuedShots;
    }

    public void armFlywheel() {
        flywheel.set(Flywheel.FlyWheelStates.ARMING, true);
    }

    public void incrementQueuedShots(int i) {
        this.queuedShots += i;
    }

    public void setQueuedShots(int i) {
        this.queuedShots = i;
    }

    public Robot.ArtifactColor getColor() {
        return feeder.getColor();
    }

    public void clearQueueShots() {
        queuedShots = 0;
        targetState = ShooterStates.IDLE;
        turret.set(Turret.TurretStates.IDLE);
        flywheel.set(Flywheel.FlyWheelStates.IDLE, true);
        feeder.set(Feeder.FeederStates.IDLE, true);
    }

    public void setGoalAlliance() {
        turret.setGoalAlliance();
    }

    public void setFeederIdle(boolean isIdle) {
        if (isIdle) feeder.set(Feeder.FeederStates.IDLE, false);
    }


    public void setTurretManual(double power) {
        turret.setManual(power);
    }

    public void applyOffsets() {
        turret.applyOffset();
    }
    public void setFlywheelManual(Flywheel.FlyWheelStates f) {
        flywheel.set(f, true);
    }

    public void setHoodManual(double angleIncrement, boolean isIncrementing) {
        hood.set(hood.get() + (isIncrementing ? angleIncrement : -angleIncrement));
    }


    @Override
    public void run() {
        didCurrentDrop = flywheel.didRPMSpike();
        if (didCurrentDrop && targetState == ShooterStates.RUNNING) {
            queuedShots--;
        }

        switch (targetState) {
            case IDLE:
                if (feeder.get() != Feeder.FeederStates.IDLE) {
                    feeder.set(Feeder.FeederStates.OFF, true);
                }

                if (queuedShots >= 1) {
                    if (flywheel.get() == Flywheel.FlyWheelStates.IDLE) flywheel.set(Flywheel.FlyWheelStates.ARMING, true);
                    targetState = ShooterStates.PREPPING;
                    if (turret.get() == Turret.TurretStates.IDLE) turret.set(Turret.TurretStates.ODOM_TRACKING, true);
                }
                break;
            case PREPPING:
                if (queuedShots >= 1 && flywheel.get() == Flywheel.FlyWheelStates.RUNNING && turret.isPIDInTolerance()) {
                    feeder.set(Feeder.FeederStates.RUNNING, true);
                    targetState = ShooterStates.RUNNING;
                    if (turret.get() == Turret.TurretStates.IDLE) turret.set(Turret.TurretStates.ODOM_TRACKING, true);
                }
                break;
            case RUNNING:
                flywheel.set(Flywheel.FlyWheelStates.RUNNING, true);
                feeder.set(Feeder.FeederStates.RUNNING, true);

                if (didCurrentDrop) {
                    if (queuedShots <= 0) {
                        targetState = ShooterStates.IDLE;
                        turret.set(Turret.TurretStates.IDLE);
                        flywheel.set(Flywheel.FlyWheelStates.IDLE, true);
                        feeder.set(Feeder.FeederStates.IDLE, true);
                    }
                    else {
                        targetState = ShooterStates.RUNNING;
                        feeder.set(Feeder.FeederStates.RUNNING, true);
                    }

                    if (turret.get() == Turret.TurretStates.IDLE) turret.set(Turret.TurretStates.ODOM_TRACKING, true);

                }

                break;
        }


        if (!isHoodManual) hood.set(hood.getHoodAngleWithDistance(turret.getDistance()), true);

        turret.run();
        flywheel.run();
        feeder.run();
        hood.run();
    }

    private double vx, vy, omega, ax, ay, alpha;
    private Pose currentPose, predictedPose;
    public Pose getPredictedPose() {
        currentPose = robot.drivetrain.getPose();

        double distanceInches = turret.getDistance();
        double rpmTarget = 0.0;
        double tLaunch = Common.LAUNCH_DELAY;
        double airtime = Common.getAirtimeForDistance(distanceInches, rpmTarget);
        double timeToShoot = tLaunch + airtime;
        vx = robot.drivetrain.getVelocity().getXComponent();
         vy = robot.drivetrain.getVelocity().getYComponent();

        omega = robot.drivetrain.getAngularVelocity();
        ax = robot.drivetrain.getAcceleration().getXComponent();
        ay = robot.drivetrain.getAcceleration().getYComponent();
        alpha = 0;

        double
                futureVx = vx + ax * timeToShoot,
                futureVy = vy + ay * timeToShoot,
                futureOmega = omega + alpha * timeToShoot,

                avgVx = (vx + futureVx) / 2.0,
                avgVy = (vy + futureVy) / 2.0,
                avgOmega = (omega + futureOmega) / 2.0,

                dx = avgVx * timeToShoot,
                dy = avgVy * timeToShoot,
                dh = avgOmega * timeToShoot;

        predictedPose = new Pose(
                currentPose.getX() + dx,
                currentPose.getY() + dy,
                currentPose.getHeading() + dh);
        return predictedPose;
    }




    @SuppressLint("DefaultLocale")
    public void printTelemetry() {
            turret.printTelemetry();
            flywheel.printTelemetry();
            feeder.printTelemetry();
            hood.printTelemetry();

        telemetry.addLine("SHOOTER");
        telemetry.addData("shooter state (ENUM):", targetState);
        telemetry.addData("queued shots (DOUBLE): ", queuedShots);
        telemetry.addData("did current drop? (BOOLEAN): ", didCurrentDrop);

        telemetry.addLine("PREDICTED POSE DEBUG");
        telemetry.addData("Velocity (vx, vy) in/s", String.format("(%.3f, %.3f)", vx, vy));
        telemetry.addData("Acceleration (ax, ay) in/s²", String.format("(%.3f, %.3f)", ax, ay));
        telemetry.addData("Angular Velocity ω (rad/s)", String.format("%.3f", omega));
        telemetry.addData("ΔPose (dx, dy, dθ°)",
                String.format("%.3f, %.3f, %.3f",
                        predictedPose.getX() - currentPose.getX(),
                        predictedPose.getY() - currentPose.getY(),
                        Math.toDegrees(predictedPose.getHeading() - currentPose.getHeading())));
        telemetry.addData("Predicted Pose (X, Y, Heading)", String.format("%.3f, %.3f, %.3f",
                        predictedPose.getX(),
                        predictedPose.getY(),
                        Math.toDegrees(predictedPose.getHeading())));
    }
}
