package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import java.util.TreeMap;

@Configurable
public class Shooter extends Subsystem<Shooter.ShooterStates> {
    private final Hood hood;
    private final Flywheel flywheel;
    private final Turret turret;

    private boolean isTracking = false;

    public enum ShooterStates {
        IDLE, PREPARING, TRACKING, RUNNING;
    }

    private ShooterStates targetState = ShooterStates.IDLE;

    private final TreeMap<Double, Double> hoodAngles = new TreeMap<>();

    public Shooter(HardwareMap hw) {
        this.hood = new Hood(hw);
        this.flywheel = new Flywheel(hw);
        this.turret = new Turret(hw);

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

    public boolean getTrackingState() {
        return isTracking;
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

    @Override
    public void run() {
        switch (targetState) {
            case IDLE: {
                flywheel.set(Flywheel.FlyWheelStates.IDLE);
                turret.set(0.0); // TODO set angle to proper 0
                hood.set(0.0); // TODO set angle to proper 0
            }
            case PREPARING: {
                flywheel.set(Flywheel.FlyWheelStates.ARMING);
                // TODO make controls to set turret angle manually
                // TODO make function to set hood angle manually
            }
            case TRACKING: {
                flywheel.set(Flywheel.FlyWheelStates.ARMING);
                isTracking = turret.setTracking(robot.drivetrain.getHeading());
                hood.set(getHoodAngleWithDistance(turret.getDistance()));
            }
            case RUNNING: flywheel.set(Flywheel.FlyWheelStates.RUNNING);
        }

    }
}
