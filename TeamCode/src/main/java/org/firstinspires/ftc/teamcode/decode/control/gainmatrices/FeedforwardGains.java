package org.firstinspires.ftc.teamcode.decode.control.gainmatrices;

public class FeedforwardGains {
    public double kV;
    public double kA;
    public double kStatic;

    public FeedforwardGains() {
        this(0.0, 0.0, 0.0);
    }

    public FeedforwardGains(double kV, double kA, double kStatic) {
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
    }
}