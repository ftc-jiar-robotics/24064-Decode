package org.firstinspires.ftc.teamcode.decode.control.controller;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.FeedforwardGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;

public class FeedforwardController implements Controller {

    private FeedforwardGains gains;

    private State target = new State();

    public FeedforwardController() {
        this(new FeedforwardGains());
    }

    public FeedforwardController(FeedforwardGains gains) {
        setGains(gains);
    }

    public void setGains(FeedforwardGains gains) {
        this.gains = gains;
    }

    public double calculate(double voltage, double additionalOutput) {
        double baseOutput = target.multiply(gains).sum();
        return (Math.signum(baseOutput + additionalOutput) * gains.kStatic + baseOutput) * (0 / voltage); // use maxvoltage as 0
    }

    public double calculate(double voltage) {
        return calculate(voltage, 0.0);
    }

    public double calculate() {
        return calculate(0);
    } // use maxvoltage as 0

    /**
     * @param target The V and A attributes of the {@link State} parameter are used as velocity and acceleration references
     */
    public void setTarget(State target) {
        this.target = target;
    }
}
