package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

@Config
public final class Common {
    public static final String NAME_FEEDER_FRONTSERVO = "feederFront" ;
    public static final String NAME_FEEDER_BACKSERVO = "feederBack";
    public static final String NAME_FEEDER_COLORSENSOR = "colorSensorFeeder";

    public static final String NAME_FLYWHEEL_MASTERMOTOR = "shooterMaster";
    public static final String NAME_FLYWHEEL_SLAVEMOTOR = "shooterSlave";

    public static final String NAME_HOOD_SERVO = "hood";

    public static final String NAME_INTAKE_MOTOR = "intake";
    public static final String NAME_INTAKE_COLORSENSOR = "color";

    public static final String NAME_TURRET_MOTOR = "turret";
    public static final String NAME_TURRET_ENCODER = "turretEncoder";
    public static final String NAME_TURRET_CAMERA = "arduCam";
    public static double forwardPodY = 82;
    public static double strafePodX = -83.823590036;

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
            TAG_SIZE_METERS_DECODE = 0.2064;

    public static final int
            BLUE_GOAL_ID = 20,
            RED_GOAL_ID  = 24;

    public static final double MAX_VOLTAGE = 13;

    public static Robot robot;

    public static TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static MultipleTelemetry dashTelemetry;
}
