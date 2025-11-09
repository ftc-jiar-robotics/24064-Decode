package org.firstinspires.ftc.teamcode.decode.control.gainmatrix;

public class FullStateGains {
    public double pGain;
    public double vGain;
    public double aGain;

    public FullStateGains() {
        this(0.0, 0.0, 0.0);
    }

    public FullStateGains(double pGain, double vGain, double aGain) {
        this.pGain = pGain;
        this.vGain = vGain;
        this.aGain = aGain;
    }
}

