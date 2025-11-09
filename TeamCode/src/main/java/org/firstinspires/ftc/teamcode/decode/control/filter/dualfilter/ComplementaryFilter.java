package org.firstinspires.ftc.teamcode.decode.control.filter.dualfilter;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.ComplementaryGains;

public final class ComplementaryFilter implements DualFilter {
    private ComplementaryGains gains;

    public ComplementaryFilter(ComplementaryGains gains) {
        setGains(gains);
    }

    public void setGains(ComplementaryGains gains) {
        this.gains = gains;
    }

    @Override
    public double calculate(double value1, double value2) {
        return value1 * gains.blend + value2 * (1 - gains.blend);
    }

    @Override
    public void reset() {}
}
