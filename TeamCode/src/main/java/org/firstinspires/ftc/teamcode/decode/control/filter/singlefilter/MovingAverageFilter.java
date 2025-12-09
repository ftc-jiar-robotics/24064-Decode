package org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.MovingAverageGains;

import java.util.ArrayList;

public final class MovingAverageFilter implements Filter {
    private MovingAverageGains gains;
    private final ArrayList<Double> buffer = new ArrayList<>();

    public MovingAverageFilter(MovingAverageGains gains) {
        setGains(gains);
    }

    public void setGains(MovingAverageGains gains) {
        this.gains = gains;
    }

    @Override
    public double calculate(double newValue) {
        buffer.add(newValue);
        if (buffer.size() > gains.count) {
            buffer.remove(0);
        } else return newValue;
        double sum = 0;
        for (double value : buffer) {
            sum += value;
        }
        return sum / gains.count;
    }

    @Override
    public void reset() {
        buffer.clear();
    }
}
