package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_BACKSERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_HOOD_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Flywheel.lutDistances;

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

    private final TreeMap<Double, Double> ayanLUT = new TreeMap<>();
    private final TreeMap<Double, Double> kashifLUT = new TreeMap<>();
    private final TreeMap<Double, Double> kayraLUT = new TreeMap<>();
    private final TreeMap<Double, Double> abucarLUT = new TreeMap<>();
    private final TreeMap<Double, Double> omarLUT = new TreeMap<>();
    
    private final TreeMap<Double, Double>[] hoodLUTS = new TreeMap[]{ayanLUT, kashifLUT, kayraLUT, abucarLUT, omarLUT};

    public Hood(HardwareMap hw) {
        this.hood = new SimpleServo(hw, NAME_HOOD_SERVO, Common.SERVO_AXON_MIN, Common.SERVO_AXON_MAX_1);

        // k = distance (inches), v = angle (deg)
        // TODO tune LUT and interpolate w/ formula
        // TODO change/tune values
        ayanLUT.put(61.0, 100.0); // RPM 3100
        ayanLUT.put(69.3, 100.5); // RPM 3100
        ayanLUT.put(80.8, 92.5); // RPM 3100
        ayanLUT.put(87.0, 91.5); // RPM 3100

        kashifLUT.put(90.2, 141.4); // RPM 3500
        kashifLUT.put(103.4, 137.4); // RPM 3500
        kashifLUT.put(110.6, 166.0); // RPM 3500
        kashifLUT.put(129.0, 166.0); // RPM 3500

        kayraLUT.put(133.0, 100.0); // RPM 3800

        abucarLUT.put(166.0, 120.0); // RPM 4100

        omarLUT.put(182.0, 130.0); // RPM 4800
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
        for (int i = 0; i < hoodLUTS.length; i++) {
            if (distance >= lutDistances[i]) lut = hoodLUTS[i];
        }

        if (lut.containsKey(distance)) return lut.get(distance);

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
        telemetry.addData("hood target angle: ", targetAngle);
        telemetry.addData("LUT output: ", lutOutput);

    }
}