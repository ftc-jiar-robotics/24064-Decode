package org.firstinspires.ftc.teamcode.decode.util;

import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.MovingAverageGains;

public final class LoopUtil {
    private static long lastTime = 0;
    private static int loops = 0;
    private static final MovingAverageFilter loopTimeFilter = new MovingAverageFilter(new MovingAverageGains(10));

    /**
     * @return The nano time between each function call
     */
    public static double getLoopTime() {
        long currentTime = System.nanoTime();
        long delta = currentTime - lastTime;
        lastTime = currentTime;
        return loopTimeFilter.calculate(delta) * 1.0;
    }

    public static void updateLoopCount() {
        loops++;
    }

    public static double getLoopTimeInSeconds() {
        return getLoopTime() / 1000000000.0;
    }

    public static double getLoopTimeInHertz() {
        return 1 / getLoopTimeInSeconds();
    }

    public static int getLoops() {
        return loops;
    }
}
