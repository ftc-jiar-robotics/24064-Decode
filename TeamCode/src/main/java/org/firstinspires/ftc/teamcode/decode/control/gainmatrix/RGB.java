package org.firstinspires.ftc.teamcode.decode.control.gainmatrix;

import static java.lang.Math.max;
import static java.lang.Math.min;

public class RGB {
    public double red;
    public double green;
    public double blue;

    public RGB() {
        this(0.0, 0.0, 0.0);
    }

    public RGB(double red, double green, double blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
    }

    public String toString(String title) {
        return String.format(title + ": \nRed: %.3f\nGreen: %.3f\nBlue: %.3f", red, green, blue);
    }

    public String toString() {
        return String.format("%.3f, %.3f, %.3f", red, green, blue);
    }
}
