package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * Shooter + ballistics constants.
 *
 * Units:
 *  - Distances: inches
 *  - Angles: radians
 */
@Configurable
public final class ShooterConstants {

    private ShooterConstants() {}
    public static Pose BLUE_GOAL = new Pose(0, 144);
    public static Pose RED_GOAL  = BLUE_GOAL.mirror();

    public static double SCORE_HEIGHT = 26.0;
    public static double SCORE_ANGLE = Math.toRadians(-30);
    public static double PASS_THROUGH_POINT_RADIUS = 5.0;
}
