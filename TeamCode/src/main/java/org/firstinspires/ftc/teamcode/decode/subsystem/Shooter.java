package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isHoodManual;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Shooter extends Subsystem<Shooter.ShooterStates> {
    public final Hood hood;
    final Flywheel flywheel;
    final Turret turret;
    final Feeder feeder;

    private boolean didCurrentDrop;

    // Used for shoot-while-moving prediction
    private static final double VEL_THRESH_INPS = 1.0;              // 1 inch/sec
    private static final double OMEGA_THRESH_RADPS = Math.toRadians(5);


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

    public void incrementQueuedShots(int i) {
        this.queuedShots += i;
    }

    public void clearQueueShots() {
        queuedShots = 0;
        targetState = ShooterStates.IDLE;
        turret.set(Turret.TurretStates.IDLE);
        flywheel.set(Flywheel.FlyWheelStates.IDLE, true);
        feeder.set(Feeder.FeederStates.IDLE, true);
    }

    public void setGoalAlliance(boolean isRed) {
        turret.setGoalAlliance(isRed);
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
        flywheel.set(f, false);
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
                    flywheel.set(Flywheel.FlyWheelStates.ARMING, true);
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

    public Pose getPredictedPose(Pose currentPose, double vx, double vy, double omega, double ax, double ay, double alpha, double timeToShoot) {

        double speed = Math.hypot(vx, vy);
        boolean moving = speed > VEL_THRESH_INPS || Math.abs(omega) > OMEGA_THRESH_RADPS;

        if (!moving) {
            return currentPose; // no movement, no prediction
        }

        // Predict velocity after the shot delay (accounts for accel if provided)
        double futureVx = vx + ax * timeToShoot;
        double futureVy = vy + ay * timeToShoot;
        double futureOmega = omega + alpha * timeToShoot;

        // Average velocities over interval
        double avgVx = (vx + futureVx) / 2.0;
        double avgVy = (vy + futureVy) / 2.0;
        double avgOmega = (omega + futureOmega) / 2.0;

        // Displacement = avg velocity × time
        double dx = avgVx * timeToShoot;
        double dy = avgVy * timeToShoot;
        double dh = avgOmega * timeToShoot;

        // Return new predicted pose
        return new Pose(
                currentPose.getX()+ dx,
                currentPose.getY() + dy,
                currentPose.getHeading() + dh
        );
    }



    public void printTelemetry() {
            turret.printTelemetry();
            flywheel.printTelemetry();
            feeder.printTelemetry();
            hood.printTelemetry();

        telemetry.addLine("SHOOTER");
        telemetry.addData("shooter state (ENUM):", targetState);
        telemetry.addData("queued shots (DOUBLE): ", queuedShots);
        telemetry.addData("did current drop? (BOOLEAN): ", didCurrentDrop);
    }
}
