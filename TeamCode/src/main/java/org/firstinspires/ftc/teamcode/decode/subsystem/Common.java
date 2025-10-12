package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

@Config
public final class Common {
    public static final String CFG_NAME_FEEDERFRONT = "feederFront" ;
    public static final String CFG_NAME_FEEDERBACK = "feederBack";
    public static final String CFG_NAME_FEEDER_COLORSENSOR = "colorSensorFeeder";
    public static Pose AUTO_END_POSE = null;

    public static boolean
            isRed = false,
            isBigTriangle = false;

    public static final double
            LEFT = Math.toRadians(180),
            FORWARD = Math.toRadians(90),
            RIGHT = Math.toRadians(0),
            BACKWARD = Math.toRadians(270),
            SERVO_25_KG_MIN = 0,
            SERVO_25_KG_MAX = 270,
            SERVO_45_KG_MIN = 0,
            SERVO_45_KG_MAX = 270,
            SERVO_AXON_MAX_1 = 270,
            SERVO_AXON_MIN = 0,
            SERVO_AXON_MAX_2 = 355,
            TAG_SIZE_METERS_DECODE = 0.2064;

    public static final int
            BLUE_GOAL_ID = 20,
            RED_GOAL_ID  = 24;

    public static final double MAX_VOLTAGE = 13;

    public static Robot robot;

    public static TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static MultipleTelemetry dashTelemetry;
    public static GraphManager graph = PanelsGraph.INSTANCE.getManager();
}
