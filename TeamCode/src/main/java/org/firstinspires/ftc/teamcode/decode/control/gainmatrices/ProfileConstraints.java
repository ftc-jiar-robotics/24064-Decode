package org.firstinspires.ftc.teamcode.decode.control.gainmatrices;

public class ProfileConstraints {
    public double maxV;
    public double maxA;
    public double maxJ;

    public ProfileConstraints() {
        this(1.0, 1.0, 0.0);
    }

    public ProfileConstraints(double maxV, double maxA, double maxJ) {
        this.maxV = maxV;
        this.maxA = maxA;
        this.maxJ = maxJ;
    }
}
