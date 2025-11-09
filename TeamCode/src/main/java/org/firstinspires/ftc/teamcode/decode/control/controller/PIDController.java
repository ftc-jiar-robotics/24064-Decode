package org.firstinspires.ftc.teamcode.decode.control.controller;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.MovingAverageGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.Differentiator;
import org.firstinspires.ftc.teamcode.decode.control.motion.Integrator;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.Filter;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.NoFilter;

public class PIDController implements FeedbackController {

    private PIDGains gains = new PIDGains();
    private State target = new State();

    private final Filter derivFilter;
    private final Differentiator errorDifferentiator = new Differentiator();
    private final Differentiator measurementDifferentiator = new Differentiator();
    private final Integrator integrator = new Integrator();

    private MovingAverageFilter movingAverageFilter = new MovingAverageFilter(new MovingAverageGains(10));

    private State error = new State();
    private double errorIntegral, filteredErrorDerivative, rawErrorDerivative;

    public PIDController() {
        this(new NoFilter());
    }

    public PIDController(Filter derivFilter) {
        this.derivFilter = derivFilter;
    }

    public void setGains(PIDGains gains) {
        this.gains = gains;
    }

    /**
     * @param measurement Only the X attribute of the {@link State} parameter is used as feedback
     */
    public double calculate(State measurement) {
        State lastError = error;
        error = target.subtract(measurement);

        if (signum(error.x) != signum(lastError.x)) reset();
        errorIntegral = integrator.getIntegral(error.x);
        rawErrorDerivative = errorDifferentiator.getDerivative(error.x);
        filteredErrorDerivative = derivFilter.calculate(rawErrorDerivative);

        double output = (gains.kP * error.x) + (gains.kI * errorIntegral) + (gains.kD * filteredErrorDerivative);

        stopIntegration(abs(output) >= gains.maxOutputWithIntegral && signum(output) == signum(error.x));

        return output;
    }

    public boolean isInTolerance(State measurement, double tolerance, double derivTolerance) {
        double deriv = movingAverageFilter.calculate(measurementDifferentiator.getDerivative(measurement.x));

        boolean isDerivativeInTolerance = Math.abs(deriv) <= derivTolerance;
        boolean isMeasurementInTolerance = Math.abs(measurement.x) <= tolerance;

        // if current has spiked and we're in tolerance and we're not in a timer(start time + time period > curr time
        return isDerivativeInTolerance && isMeasurementInTolerance;
    }

    public boolean isInTolerance(State measurement, double tolerance) {
        return isInTolerance(measurement, tolerance, Double.POSITIVE_INFINITY);
    }

    public void setTarget(State target) {
        this.target = target;
    }

    public double getFilteredErrorDerivative() {
        return filteredErrorDerivative;
    }

    public double getRawErrorDerivative() {
        return rawErrorDerivative;
    }

    public double getErrorIntegral() {
        return errorIntegral;
    }

    public double getError() {return error.x;}

    public void stopIntegration(boolean stopIntegration) {
        integrator.stopIntegration(stopIntegration);
    }

    public void reset() {
        integrator.reset();
        derivFilter.reset();
    }
}
