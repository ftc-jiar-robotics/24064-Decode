package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class ShooterConstants {
    public static Pose
            BLUE_GOAL = new Pose(0, 144),
            RED_GOAL = BLUE_GOAL.mirror();

    public static double
            SCORE_HEIGHT = 26, //inches
            SCORE_ANGLE = Math.toRadians(-30), //radians
            PASS_THROUGH_POINT_RADIUS = 5; //inches
}
