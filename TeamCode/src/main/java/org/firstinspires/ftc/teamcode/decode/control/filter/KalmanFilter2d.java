package org.firstinspires.ftc.teamcode.decode.control.filter;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.KalmanGains;
import org.firstinspires.ftc.teamcode.decode.control.filter.dualfilter.KalmanFilter;

// Credits to @j5155 of Capital City Dynamics 12087
// Released under BSD-3-Clause-Clear license
public class KalmanFilter2d {
    private final KalmanFilter x;
    private final KalmanFilter y;

    public KalmanFilter2d(KalmanFilter x, KalmanFilter y) {
        this.x = x;
        this.y = y;
    }

    public KalmanFilter2d(double Q, double R) {
        this.x = new KalmanFilter(new KalmanGains(Q, R));
        this.y = new KalmanFilter(new KalmanGains(Q, R));
    }

    /**
     * @param model the CHANGE since last update.
     */
    public Vector2d update(Twist2d model, Pose2d sensor) {
        Pose2d modelPose = Pose2d.exp(model);
        return new Vector2d(
                x.calculate(modelPose.position.x, sensor.position.x),
                y.calculate(modelPose.position.y, sensor.position.y)
        );
    }

    /**
     * @param model the CHANGE since last update.
     */
    public Vector2d update(Twist2d model, Vector2d sensor) {
        return new Vector2d(
                x.calculate(model.line.x, sensor.x),
                y.calculate(model.line.y, sensor.y));
    }

    public KalmanFilter getXFilter() {
        return x;
    }

    public KalmanFilter getYFilter() {
        return y;
    }
}
