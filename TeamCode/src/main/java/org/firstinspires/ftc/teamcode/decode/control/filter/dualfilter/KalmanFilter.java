package org.firstinspires.ftc.teamcode.decode.control.filter.dualfilter;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.KalmanGains;

public final class KalmanFilter implements DualFilter {
    private KalmanGains gains;
    private double x = 0; // Your initial state
    private double p = 1; // Your initial covariance guess
    private double K = 1; // Your initial Kalman gain guess
    private double xPrevious = x;
    private double pPrevious = p;
    private double u = 0;
    private double z = 0;

    public KalmanFilter(KalmanGains gains) {
        setGains(gains);
    }

    public void setGains(KalmanGains gains) {
        this.gains = gains;
    }

    /**
     * @param model the CHANGE in state from the model
     * @param sensor the state from the sensor (i.e. April Tag / distance sensor)
     * @return the filtered Kalman state
     */
    @Override
    public double calculate(double model, double sensor) {
        u = model; // Change in position from odometry.
        x = xPrevious + u;

        p = pPrevious + gains.Q;

        K = p / (p + gains.R);

        z = sensor; // Pose estimate from April Tag / distance sensor

        x = x + K * (z - x);

        p = (1 - K) * p;

        xPrevious = x;
        pPrevious = p;
        return x;
    }

    @Override
    public void reset() {
        x = 0;
        p = K = 1;
        xPrevious = x;
        pPrevious = p;
    }
}
