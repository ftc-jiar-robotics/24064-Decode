package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

@Config
public final class Common {
    public static final String
            NAME_FEEDER_FRONT_SERVO = "feederFront" ,
            NAME_FEEDER_BACK_SERVO = "feederBack",
            NAME_FEEDER_COLOR_SENSOR = "colorSensorFeeder",

            NAME_FLYWHEEL_MASTER_MOTOR = "shooterMaster",
            NAME_FLYWHEEL_SLAVE_MOTOR = "shooterSlave",

            NAME_HOOD_SERVO = "hood",

            NAME_INTAKE_MOTOR = "intake",
            NAME_INTAKE_COLOR_SENSOR = "color",

            NAME_TURRET_MOTOR = "turret",
            NAME_TURRET_ENCODER = "turretEncoder",
            NAME_TURRET_CAMERA = "arduCam";

    public static Pose AUTO_END_POSE = null;

    public static boolean
            isSlowMode = false,
            isTelemetryOn = false,
            isRed = false,
            isBigTriangle = false,
            isHoodManual = false;

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

            FORWARD_POD_Y = 80,
            STRAFE_POD_X = -83.5,

            // Camera Stuff
            CAM_HEIGHT = 11.5,
            CAM_PITCH = 10.0,

            // Camera offset from turret center (inches) +X = right, -X = left | +Y = forward, -Y = backward
            CAM_OFFSET_X = -2.2,
            CAM_OFFSET_Y = 6,
            TURRET_OFFSET_Y = -2.559,

            MAX_VOLTAGE = 13;

    public static double TIME_TO_SHOOT = 0.2;
    public static final int
            BLUE_GOAL_ID = 20,
            RED_GOAL_ID  = 24;

    public static Robot robot;

    public static TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static MultipleTelemetry dashTelemetry;
}
