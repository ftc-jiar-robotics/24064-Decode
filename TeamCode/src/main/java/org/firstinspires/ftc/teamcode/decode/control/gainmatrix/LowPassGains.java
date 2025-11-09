package org.firstinspires.ftc.teamcode.decode.control.gainmatrix;

public class LowPassGains {
    public double gain;
    public int count;

    public LowPassGains(double gain, int count) {
        this.gain = gain;
        this.count = count;
    }

    public LowPassGains(double gain) {
        this(gain, 2);
    }

    public LowPassGains() {
        this(0.0, 2);
    }
}
