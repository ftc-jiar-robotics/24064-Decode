package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import android.util.Log;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.TreeMap;

@Configurable
public class Hood extends Subsystem<Double> {
    private final SimpleServo hood;

    private double targetAngle;

    public static double
            PHYSICAL_MAX = 200,
            MAX = 180,
            MIN = 75;

    private final TreeMap<Double, Double> hoodAngles = new TreeMap<>();

    public Hood(HardwareMap hw) {
        this.hood = new SimpleServo(hw, "hood", Common.SERVO_AXON_MIN, Common.SERVO_AXON_MAX_1);

        // k = distance (inches), v = angle (deg)
        // TODO tune LUT and interpolate w/ formula
        // TODO change/tune values
        hoodAngles.put(54.0, 75.0); // RPM 3500
        hoodAngles.put(60.5, 87.0); // RPM 3500
        hoodAngles.put(67.0, 105.0); // RPM 3500
        hoodAngles.put(73.49, 117.5); // RPM 3500
        hoodAngles.put(73.5, 145.0); // RPM 4000
    }

    @Override
    public Double get() {
        return targetAngle;
    }

    @Override
    public void set(Double a) {
        targetAngle = Range.clip(a, MIN, MAX);
    }

    public void setPhysicalMax() {
        targetAngle = PHYSICAL_MAX;
    }

    public double getHoodAngleWithDistance(double distance) {
        if (hoodAngles.containsKey(distance)) return hoodAngles.get(distance);

        double finalDistance = Range.clip(distance, hoodAngles.firstKey(), hoodAngles.lastKey());

        if (finalDistance >= hoodAngles.firstKey() && finalDistance <= hoodAngles.lastKey()) {
            double x2 = hoodAngles.ceilingKey(finalDistance); // x2
            double x1 = hoodAngles.floorKey(finalDistance); // x1

            double y2 = hoodAngles.get(x2); // y2
            double y1 = hoodAngles.get(x1); // y1

            if (x2 == x1) return get();

            return Range.clip(y1 + ((finalDistance - x1) * (y2 - y1)) / (x2 - x1), MIN, MAX);
        }

        return get();
    }

    @Override
    public void run() {
        hood.turnToAngle(Range.clip(targetAngle, MIN, PHYSICAL_MAX));
    }

    public void printTelemetry() {
        telemetry.addLine("HOOD");
        telemetry.addData("hood target angle: ", targetAngle);
    }
}