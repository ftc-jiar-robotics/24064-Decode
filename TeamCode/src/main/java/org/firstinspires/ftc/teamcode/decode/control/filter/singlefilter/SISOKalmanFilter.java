package org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.KalmanGains;

public final class SISOKalmanFilter implements Filter {

    private KalmanGains gains;
    private final boolean wraparound;

    public SISOKalmanFilter() {
        this(new KalmanGains());
    }
    public SISOKalmanFilter(KalmanGains gains) {
        this(gains, false);
    }
    public SISOKalmanFilter(KalmanGains gains, boolean wraparound) {
        setGains(gains);
        this.wraparound = wraparound;
    }

    public void setGains(KalmanGains gains) {
        this.gains = gains;
    }

    private double x = 0; // your initial state
    private double p = 1; // your initial covariance guess
    private double K = 1; // your initial Kalman gain guess
    public double x_previous = x;
    private double p_previous = p;
    private double u = 0;
    private double z = 0;

    public void reset() {
        x = 0;
        p = 1;
        K = 1;
        x_previous = x;
        p_previous = p;
        u = 0;
        z = 0;
    }

    @Override
    public double calculate(double newValue) {

        if (wraparound)
            newValue = x_previous + normalizeRadians(newValue - x_previous);

        u = 0;
        // Predict our new state
        x = x_previous + u;

        // Predict our new covariance
        p = p_previous + gains.Q;

        // Update our kalman gain
        K = p/(p + gains.R);

        // Get a new reading from sensor
        z = newValue; // Pose Estimate from April Tag / Distance Sensor

        // Update the state
        x = x + K * (z - x);

        // Update the covariance
        p = (1 - K) * p;

        x_previous = x;
        p_previous = p;

        return x;

    }
}