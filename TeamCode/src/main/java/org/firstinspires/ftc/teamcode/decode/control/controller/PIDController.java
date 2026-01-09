package org.firstinspires.ftc.teamcode.decode.control.controller;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.MovingAverageGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.Differentiator;
import org.firstinspires.ftc.teamcode.decode.control.motion.Integrator;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.Filter;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.NoFilter;

public class PIDController implements FeedbackController {

    public enum DerivativeMode {
        ERROR,        // current behavior (default)
        MEASUREMENT   // for flywheel (no derivative kick from setpoint noise)
    }

    private DerivativeMode derivativeMode = DerivativeMode.ERROR;

    public void setDerivativeMode(DerivativeMode mode) {
        this.derivativeMode = mode;
    }


    private PIDGains gains = new PIDGains();
    private State target = new State();

    private final Filter derivFilter;
    private final Differentiator errorDifferentiator = new Differentiator();
    private final Differentiator measurementDifferentiator = new Differentiator();
    private final Integrator integrator = new Integrator();

    private MovingAverageFilter movingAverageFilter = new MovingAverageFilter(new MovingAverageGains(3));

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
        if (derivativeMode == DerivativeMode.MEASUREMENT) {
            // D on measurement (prevents derivative kick from noisy setpoint)
            rawErrorDerivative = measurementDifferentiator.getDerivative(measurement.x);
        } else {
            // Default: D on error (existing behavior)
            rawErrorDerivative = errorDifferentiator.getDerivative(error.x);
        }
        filteredErrorDerivative = derivFilter.calculate(rawErrorDerivative);

        double dTerm = (derivativeMode == DerivativeMode.MEASUREMENT) ? (-gains.kD * filteredErrorDerivative) : (gains.kD * filteredErrorDerivative);

        double output = (gains.kP * error.x) + (gains.kI * errorIntegral) + dTerm;



        stopIntegration(abs(output) >= gains.maxOutputWithIntegral && signum(output) == signum(error.x));

        return output;
    }

    public boolean isInTolerance(State measurement, double tolerance, double derivTolerance) {
        double rawDeriv = (derivativeMode == DerivativeMode.MEASUREMENT) ? rawErrorDerivative : measurementDifferentiator.getDerivative(measurement.x);
// already d(measurement)/dt from calculate()
        double deriv = movingAverageFilter.calculate(rawDeriv);

        boolean isDerivativeInTolerance = Math.abs(deriv) <= derivTolerance;
        boolean isMeasurementInTolerance = Math.abs(target.x - measurement.x) <= tolerance;

        return isDerivativeInTolerance && isMeasurementInTolerance;
    }


    public boolean isInTolerance(State measurement, double tolerance) {
        return Math.abs(target.x - measurement.x) <= tolerance;
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
