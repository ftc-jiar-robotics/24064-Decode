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

    public HSV toHSV() {
        // R, G, B values are divided by 255
        // to change the range from 0..255 to 0..1
        double r = red / 255.0;
        double g = green / 255.0;
        double b = blue / 255.0;

        double colorMax = max(r, max(g, b)); // maximum of r, g, b

        double colorMin = min(r, min(g, b)); // minimum of r, g, b

        double diff = colorMax - colorMin; // diff of cmax and cmin.

        return new HSV(
                colorMax == colorMin ? 0.0 :
                        colorMax == r ? (60 * ((g - b) / diff) + 360) % 360 :
                                colorMax == g ? (60 * ((b - r) / diff) + 120) % 360 :
                                        colorMax == b ? (60 * ((r - g) / diff) + 240) % 360 :
                                                0.0,
                colorMax == 0.0 ? 0.0 : diff / colorMax,
                colorMax
        );
    }
}
