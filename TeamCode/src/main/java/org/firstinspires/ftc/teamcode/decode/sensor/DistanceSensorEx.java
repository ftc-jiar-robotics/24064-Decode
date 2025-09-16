package org.firstinspires.ftc.teamcode.decode.sensor;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.LowPassGains;

public class DistanceSensorEx {
    private final DistanceSensor distanceSensor;

    private final FIRLowPassFilter firLowPassFilter;

    public DistanceSensorEx(DistanceSensor distanceSensor) {
        this.distanceSensor = distanceSensor;

        firLowPassFilter = new FIRLowPassFilter();

        firLowPassFilter.setGains(new LowPassGains(0, 2));
    }

    public double calculateDistance() {
        return firLowPassFilter.calculate(distanceSensor.getDistance(DistanceUnit.INCH));
    }
}
