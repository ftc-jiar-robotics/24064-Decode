package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.TreeMap;

@Configurable
public class Hood extends Subsystem<Double> {
    private final SimpleServo hood;

    private double
            targetAngle,
            lutOutput;

    public static double
            PHYSICAL_MAX = 200,
            MAX = 180,
            MIN = 68;

    private final TreeMap<Double, Double> closeLUT = new TreeMap<>();
    private final TreeMap<Double, Double> farLUT = new TreeMap<>();

    public Hood(HardwareMap hw) {
        this.hood = new SimpleServo(hw, "hood", Common.SERVO_AXON_MIN, Common.SERVO_AXON_MAX_1);

        // k = distance (inches), v = angle (deg)
        // TODO tune LUT and interpolate w/ formula
        // TODO change/tune values
        closeLUT.put(60.5, 103.5); // RPM 3890
        closeLUT.put(72.0, 120.5); // RPM 3890
        closeLUT.put(79.5, 115.5); // RPM 3890
        closeLUT.put(88.5, 124.0); // RPM 3890



        farLUT.put(97.5, 164.5); // 4300
        farLUT.put(104.0, 157.5); // 4300
        farLUT.put(117.0, 166.0); // 4300

        farLUT.put(129.0, 166.0); // 4400

        farLUT.put(149.0, 140.0); // 4400
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
        TreeMap<Double, Double> LUT = distance >= Flywheel.RPM_CHANGE_DISTANCE ? farLUT : closeLUT;

        if (LUT.containsKey(distance)) return LUT.get(distance);

        double finalDistance = Range.clip(distance, LUT.firstKey(), LUT.lastKey());

        if (finalDistance >= LUT.firstKey() && finalDistance <= LUT.lastKey()) {
            double x2 = LUT.ceilingKey(finalDistance); // x2
            double x1 = LUT.floorKey(finalDistance); // x1

            double y2 = LUT.get(x2); // y2
            double y1 = LUT.get(x1); // y1

            if (x2 == x1) return get();

            lutOutput = Range.clip(y1 + ((finalDistance - x1) * (y2 - y1)) / (x2 - x1), MIN, MAX);

            return lutOutput;
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
        telemetry.addData("LUT output: ", lutOutput);

    }
}