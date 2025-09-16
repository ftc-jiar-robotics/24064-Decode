package org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter;

public interface Filter {

    default double calculate(double newValue)  {
        return newValue;
    }

    default void reset() {}
}
