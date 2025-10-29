package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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


    //Camera Stuff
        // Height of the ArduCam lens from the ground (inches)
            CAM_HEIGHT = 9.8,
        //Camera pitch (tilt upward from horizontal, radians)
            CAM_PITCH = Math.toRadians(5.0),
        //Camera offset from turret center (inches)
        //+X = right, -X = left | +Y = forward, -Y = backward
            CAM_OFFSET_X = 0,
            CAM_OFFSET_Y =0,

    //Goal Tings
            GOAL_FROM_TAG_X = 0,
            GOAL_FROM_TAG_Y =0,

    //Turret Stuff
        //Distance from robot center to turret center (inches)
        // should be negative.
            TURRET_OFFSET_Y = -4.0,
        //Flywheel offset from camera (inches)
        // +X = right, +Y = forward
            FLYWHEEL_OFFSET_X = 0.0,
            FLYWHEEL_OFFSET_Y = 0.0,
    //
            TAG_SIZE_METERS_DECODE = 0;

    // {tagId, tagX, tagY, tagHeight}
    public static final double [][] TAG_POSES = {
            {20, 0.0, -72.0, 42.0}, //Blue Goal
            {24, 0.0, 72.0, 42.0}   //Red Goal
    };



    public static final int
            BLUE_GOAL_ID = 20,
            RED_GOAL_ID  = 24;

    public static final double MAX_VOLTAGE = 13;

    public static Robot robot;

    public static TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static MultipleTelemetry dashTelemetry;
}
