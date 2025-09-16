package org.firstinspires.ftc.teamcode.decode.control.gainmatrices;

import kotlin.jvm.JvmField;
import kotlin.jvm.JvmOverloads;

import static java.lang.Math.*;

public class PIDGains {

    @JvmField
    public double kP;
    @JvmField
    public double kI;
    @JvmField
    public double kD;
    @JvmField
    public double maxOutputWithIntegral;

    @JvmOverloads
    public PIDGains(double kP, double kI, double kD, double maxOutputWithIntegral) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxOutputWithIntegral = maxOutputWithIntegral;
    }

    @JvmOverloads
    public PIDGains(double kP, double kI, double kD) {
        this(kP, kI, kD, Double.POSITIVE_INFINITY);
    }

    @JvmOverloads
    public PIDGains(double kP, double kI) {
        this(kP, kI, 0.0, Double.POSITIVE_INFINITY);
    }

    @JvmOverloads
    public PIDGains(double kP) {
        this(kP, 0.0, 0.0, Double.POSITIVE_INFINITY);
    }

    @JvmOverloads
    public PIDGains() {
        this(0.0, 0.0, 0.0, Double.POSITIVE_INFINITY);
    }


    @JvmOverloads
    public PIDGains computeKd(FeedforwardGains gains, double percentOvershoot) {
        if (percentOvershoot == 0.0) {
            this.kD = computeKd(kP, gains.kV, gains.kA);
        } else {
            this.kD = computeKd(kP, gains.kV, gains.kA, percentOvershoot);
        }
        return this;
    }

    public PIDGains computeKd(FeedforwardGains gains) {
        return computeKd(gains, 0.0);
    }

    @JvmOverloads
    public static double computeKd(double kP, double kV, double kA, double percentOvershoot) {
        double overshoot = percentOvershoot / 100.0;
        double zeta = (overshoot <= 0.0) ? 1.0 : -log(overshoot) / sqrt(pow(PI, 2) + pow(log(overshoot), 2));
        return max(
            2 * zeta * sqrt(kA * kP) - kV,
            0.0
        );
    }

    public static double computeKd(double kP, double kV, double kA) {
        return computeKd(kP, kV, kA, 0.0);
    }
}
