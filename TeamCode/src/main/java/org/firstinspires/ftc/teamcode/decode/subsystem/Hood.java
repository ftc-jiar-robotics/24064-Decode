package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_HOOD_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Flywheel.lutDistances;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.util.CachedServo;

import java.util.TreeMap;

@Configurable
public class Hood extends Subsystem<Double> {
    private final CachedServo hood;

    private double
            targetAngle,
            lutOutput;

    public static double
            PHYSICAL_MAX = 200,
            MAX = 180,
            MIN = 68;

    private final TreeMap<Double, Double> ayanLUT = new TreeMap<>();
    private final TreeMap<Double, Double> kashifLUT = new TreeMap<>();
    private final TreeMap<Double, Double> kayraLUT = new TreeMap<>();
    private final TreeMap<Double, Double> abucarLUT = new TreeMap<>();
    private final TreeMap<Double, Double> omarLUT = new TreeMap<>();
    
    private final TreeMap<Double, Double>[] hoodLUTS = new TreeMap[]{ayanLUT, kashifLUT, kayraLUT, abucarLUT, omarLUT};

    public Hood(HardwareMap hw) {
        this.hood = new CachedServo(hw, NAME_HOOD_SERVO, Common.SERVO_AXON_MIN, Common.SERVO_AXON_MAX_1, AngleUnit.DEGREES);

        // k = distance (inches), v = angle (deg)
        // TODO tune LUT and interpolate w/ formula
        // TODO change/tune values
        ayanLUT.put(64.0, 108.0); // RPM 3100
        ayanLUT.put(68.2, 116.0); // RPM
        ayanLUT.put(78.0, 99.5); // RPM 3100
        ayanLUT.put(81.0, 112.5); // RPM 3100

        kashifLUT.put(81.0, 139.0); // RPM 3500
        kashifLUT.put(89.0, 152.5); // RPM 3500
        kashifLUT.put(103.0, 139.0); // RPM 3500
        kashifLUT.put(115.0, 136.0); // RPM 3500


        kayraLUT.put(115.0, 156.0); // RPM 3800
        kayraLUT.put(130.0, 153.0); // RPM 3800
        kayraLUT.put(135.0, 168.0); // RPM 3800
        kayraLUT.put(143.0, 169.0); // RPM 3800
        kayraLUT.put(155.0, 177.0); // RPM 3800

        abucarLUT.put(166.0, 130.0); // RPM 4100

        omarLUT.put(182.0, 140.0); // RPM 4800
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
        TreeMap<Double, Double> lut;

        lut = hoodLUTS[0];
        int index = 0;
        for (; index < hoodLUTS.length; index++) {
            if (distance >= lutDistances[index]) lut = hoodLUTS[index];
        }

        if (lut.containsKey(distance)) return lut.get(distance);

        TreeMap<Double, Double> nextLUT = hoodLUTS[Range.clip(index + 1, 0, hoodLUTS.length - 1)];

        if (distance > lut.lastKey() && distance < nextLUT.firstKey()) {
            double x2 = nextLUT.ceilingKey(distance); // x2
            double x1 = lut.floorKey(distance); // x1

            double y2 = nextLUT.get(x2); // y2
            double y1 = lut.get(x1); // y1

            if (x2 == x1) return get();

            lutOutput = Range.clip(y1 + ((distance - x1) * (y2 - y1)) / (x2 - x1), MIN, MAX);

            return lutOutput;
        }

        double finalDistance = Range.clip(distance, lut.firstKey(), lut.lastKey());

        if (finalDistance >= lut.firstKey() && finalDistance <= lut.lastKey()) {
            double x2 = lut.ceilingKey(finalDistance); // x2
            double x1 = lut.floorKey(finalDistance); // x1

            double y2 = lut.get(x2); // y2
            double y1 = lut.get(x1); // y1

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
        telemetry.addData("hood target angle (ANGLE): ", targetAngle);
        telemetry.addData("LUT output (INCHES): ", lutOutput);

    }
}