package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Shooter extends Subsystem<Shooter.ShooterStates> {
    public final Hood hood; //TODO make un public
    final Flywheel flywheel;
    final Turret turret;
    final Feeder feeder;

    private boolean didCurrentDrop;

    private int queuedShots = 0;

    public enum ShooterStates {
        IDLE, MANUAL, TRACKING, RUNNING;
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

    // TODO add motor pid checks
    public boolean isShooterReady() {
        return targetState == ShooterStates.RUNNING;
    }

    public void runManual(double power) {
        turret.setManualPower(power);
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

    @Override
    public void run() {
        // TODO add voltage readings for flywheel & decrement queued shots
        didCurrentDrop = flywheel.didCurrentSpike();
        if (didCurrentDrop && targetState == ShooterStates.RUNNING) {
            queuedShots--;
        }

        switch (targetState) {
            case IDLE:
                feeder.set(Feeder.FeederStates.OFF);
                flywheel.set(Flywheel.FlyWheelStates.IDLE);
                turret.set(Turret.TurretStates.ODOM_TRACKING);
//                hood.set(hood.MIN);

                if (queuedShots >= 1) {
                    flywheel.set(Flywheel.FlyWheelStates.ARMING);
                    targetState = ShooterStates.TRACKING;
                }
                break;
            case MANUAL:
                feeder.set(Feeder.FeederStates.OFF);
                flywheel.set(Flywheel.FlyWheelStates.ARMING);
                // TODO make controls to set turret angle manually
                // TODO make function to set hood angle manually
                break;
            case TRACKING:
                feeder.set(Feeder.FeederStates.OFF);
                turret.set(Turret.TurretStates.ODOM_TRACKING);
//                hood.set(hood.getHoodAngleWithDistance(turret.getDistance()));

                // TODO add checks for all PIDS
                if (queuedShots >= 1 && flywheel.get() == Flywheel.FlyWheelStates.RUNNING && turret.isPIDInTolerance()) {
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
