package org.firstinspires.ftc.teamcode.decode.control.gainmatrices;

public class KalmanGains {
    public double Q; // Trust in model
    public double R; // Trust in sensor

    public KalmanGains() {
        this(0.1, 0.4);
    }

    public KalmanGains(double Q, double R) {
        this.Q = Q;
        this.R = R;
    }
}