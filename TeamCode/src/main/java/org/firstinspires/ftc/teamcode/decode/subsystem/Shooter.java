package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.TreeMap;

@Configurable
public class Shooter extends Subsystem<Shooter.ShooterStates> {
    public final Hood hood;
    private final Flywheel flywheel;
    private final Turret turret;
    private final Feeder feeder;

    private int queuedShots = 0;

    public enum ShooterStates {
        IDLE, MANUAL, TRACKING, RUNNING;
    }

    private ShooterStates targetState = ShooterStates.IDLE;

    private final TreeMap<Double, Double> hoodAngles = new TreeMap<>();

    public Shooter(HardwareMap hw) {
        this.hood = new Hood(hw);
        this.flywheel = new Flywheel(hw);
        this.turret = new Turret(hw);
        this.feeder = new Feeder(hw);

        // k = distance (ft), v = angle (deg)
        // TODO tune LUT and interpolate w/ formula
        // TODO change/tune values
        hoodAngles.put(3.0, 2.0);
        hoodAngles.put(4.0, 7.0);
        hoodAngles.put(5.0, 12.0);
        hoodAngles.put(6.0, 17.0);
        hoodAngles.put(7.0, 22.0);
        hoodAngles.put(8.0, 27.0);
        hoodAngles.put(9.0, 32.0);
        hoodAngles.put(10.0, 35.0);
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

    public double getHoodAngleWithDistance(double distance) {
        if (hoodAngles.containsKey(distance)) return hoodAngles.get(distance);

        double finalDistance = Range.clip(distance, hoodAngles.firstKey(), hoodAngles.lastKey());

        if (finalDistance >= hoodAngles.firstKey() && finalDistance <= hoodAngles.lastKey()) {
            double x2 = hoodAngles.ceilingKey(finalDistance); // x2
            double x1 = hoodAngles.floorKey(finalDistance); // x1

            double y2 = hoodAngles.get(x2); // y2
            double y1 = hoodAngles.get(x1); // y1

            return y1 + ((finalDistance - x1) * (y2 - y1)) / (x2 - x1);
        }

        return hood.get();
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

    @Override
    public void run() {
        // TODO add voltage readings for flywheel & decrement queued shots
        boolean didCurrentDrop = flywheel.didCurrentDrop();
        if (didCurrentDrop) {
            queuedShots--;
        }

        switch (targetState) {
            case IDLE:
                feeder.set(Feeder.FeederStates.OFF);
                flywheel.set(Flywheel.FlyWheelStates.IDLE);
                turret.set(Turret.TurretStates.ODOM_TRACKING); // TODO set angle to proper 0
                hood.set(0.0); // TODO set angle to proper 0

                if (queuedShots >= 1) targetState = ShooterStates.TRACKING;
                break;
            case MANUAL:
                feeder.set(Feeder.FeederStates.OFF);
                flywheel.set(Flywheel.FlyWheelStates.ARMING);
                // TODO make controls to set turret angle manually
                // TODO make function to set hood angle manually
                break;
            case TRACKING:
                feeder.set(Feeder.FeederStates.OFF);
                flywheel.set(Flywheel.FlyWheelStates.ARMING);
                turret.set(Turret.TurretStates.ODOM_TRACKING);
                hood.set(getHoodAngleWithDistance(turret.getDistance()));

                // TODO add checks for all PIDS
                if (queuedShots >= 1 && flywheel.isPIDInTolerance() && turret.isPIDInTolerance()) targetState = ShooterStates.RUNNING;
                break;
            case RUNNING:
                flywheel.set(Flywheel.FlyWheelStates.RUNNING);
                feeder.set(Feeder.FeederStates.RUNNING);

                if (didCurrentDrop) { // TODO needs to happen when voltage drop happens
                    if (queuedShots == 0) targetState = ShooterStates.IDLE;
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
    }
}
