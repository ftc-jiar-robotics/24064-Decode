package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Shooter extends Subsystem<Shooter.ShooterStates> {
    public final Hood hood; //TODO make un public remove manual control from main teleop
    final Flywheel flywheel;
    final Turret turret;
    final Feeder feeder;

    private boolean didCurrentDrop;
    private int queuedShots = 0;

    public enum ShooterStates {
        IDLE, TRACKING, RUNNING;
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

    // TODO make decrement available only to decrement action
    public void decrementQueuedShots() {
        this.queuedShots--;
    }

    public void clearQueueShots() {
        queuedShots = 0;
    }

    public void setFeederManual(double powerFront, double powerBack) {
        feeder.set(powerFront, powerBack);
    }

    @Override
    public void run() {
        // TODO add voltage readings for flywheel & decrement queued shots
        didCurrentDrop = flywheel.didCurrentSpike();
        if (didCurrentDrop && targetState == ShooterStates.RUNNING) {
            queuedShots--;
        }

        switch (targetState) {
            case IDLE:
                if (feeder.get() != Feeder.FeederStates.IDLE) {
                    feeder.set(Feeder.FeederStates.OFF);
                }

                flywheel.set(Flywheel.FlyWheelStates.IDLE);
                turret.set(Turret.TurretStates.ODOM_TRACKING);
//                hood.set(hood.MIN);

                if (queuedShots >= 1) {
                    flywheel.set(Flywheel.FlyWheelStates.ARMING);
                    targetState = ShooterStates.TRACKING;
                }
                break;
            case TRACKING:
                feeder.set(Feeder.FeederStates.OFF);
                turret.set(Turret.TurretStates.ODOM_TRACKING);

                // TODO add checks for all PIDS
                if (queuedShots >= 1 && flywheel.get() == Flywheel.FlyWheelStates.RUNNING && turret.isPIDInTolerance()) {
//                    hood.set(hood.getHoodAngleWithDistance(turret.getDistance()));
                    feeder.set(Feeder.FeederStates.RUNNING);
                    targetState = ShooterStates.RUNNING;
                }
                break;
            case RUNNING:
                flywheel.set(Flywheel.FlyWheelStates.RUNNING);

                if (didCurrentDrop) { // TODO needs to happen when voltage drop happens
                    if (queuedShots <= 0) targetState = ShooterStates.IDLE;
                    else targetState = ShooterStates.TRACKING;

                    feeder.set(Feeder.FeederStates.OFF);
                }

                break;
        }

        turret.run();
        flywheel.run();
        feeder.run();
        hood.run();
    }

    public void printTelemetry() {
        turret.printTelemetry();
        flywheel.printTelemetry();
        feeder.printTelemetry();
        hood.printTelemetry();

        telemetry.addLine("SHOOTER");
        telemetry.addData("shooter state:", targetState);
        telemetry.addData("queued shots: ", queuedShots);
        telemetry.addData("did current drop?: ", didCurrentDrop);
    }
}
