package org.firstinspires.ftc.teamcode.decode.control.motion;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.FeedforwardGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.FullStateGains;

public class State {
    public double x;
    public double v;
    public double a;
    public double j;

    public State() {
        this(0.0, 0.0, 0.0, 0.0);
    }

    public State(double x, double v, double a, double j) {
        this.x = x;
        this.v = v;
        this.a = a;
        this.j = j;
    }

    public State add(State other) {
        return new State(
                this.x + other.x,
        this.v + other.v,
        this.a + other.a,
        this.j + other.j
        );
    }

    public State negate() {
        return new State(
                -this.x,
        -this.v,
        -this.a,
        -this.j
        );
    }

    public State subtract(State other) {
        return this.add(other.negate());
    }

    public State multiply(FullStateGains gains) {
        return new State(
                this.x * gains.pGain,
        this.v * gains.vGain,
        this.a * gains.aGain,
        0.0
        );
    }

    public State multiply(FeedforwardGains gains) {
        return new State(
                0.0,
        this.v * gains.kV,
        this.a * gains.kA,
        0.0
        );
    }

    /**
     * Returns the sum of all the derivatives of this State.
     * Only to be used to get a control output.
     */
    public double sum() {
        return x + v + a + j;
    }
}
