package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.HSV;

@Config
public final class Common {
    public static final String
            NAME_FEEDER_FRONT_SERVO = "feederFront" ,
            NAME_FEEDER_BACK_SERVO = "feederBack",
            NAME_FEEDER_COLOR_SENSOR = "colorSensorFeeder",
            NAME_FEEDER_LEFT_DISTANCE_SENSOR = "leftDistanceSensorFeeder",
            NAME_FEEDER_RIGHT_DISTANCE_SENSOR = "rightDistanceSensorFeeder",
            NAME_FEEDER_LEFT_PIN0 = "leftDistancePin0",
            NAME_FEEDER_RIGHT_PIN0 = "rightDistancePin0",

            NAME_FLYWHEEL_MASTER_MOTOR = "shooterMaster",
            NAME_FLYWHEEL_SLAVE_MOTOR = "shooterSlave",

            NAME_HOOD_SERVO = "hood",

            NAME_INTAKE_MOTOR = "intake",
            NAME_INTAKE_COLOR_SENSOR = "colorSensorIntake",

            NAME_TURRET_MOTOR = "turret",
            NAME_TURRET_ENCODER = "turretEncoder",
            NAME_TURRET_CAMERA = "arduCam";

    public static Pose AUTO_END_POSE = null;

    public static boolean
            isFuturePoseOn = false,
            isSlowMode = false,
            isTelemetryOn = false,
            isRed = false,
            isBigTriangle = false,
            isHoodManual = false,
            isFlywheelManual = false,
            inTriangle = false;

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
            CAM_HEIGHT = 14.25,
            CAM_PITCH = 20.0,

            // Camera offset from turret center (inches) +X = right, -X = left | +Y = forward, -Y = backward
            CAM_OFFSET_X = -1.254,
            CAM_OFFSET_Y = -3.281,
            TURRET_OFFSET_Y = -2.559,

            MAX_VOLTAGE = 13;

    public static final Pose
            RED_BIG_TRIANGLE = new Pose(113.5, 135.5, Math.toRadians(270)),
            RED_SMALL_TRIANGLE = new Pose(88.5, 7.25, Math.toRadians(90)),
            BLUE_BIG_TRIANGLE = RED_BIG_TRIANGLE.mirror().setHeading((3.0 * Math.PI) / 2.0),
            BLUE_SMALL_TRIANGLE = RED_SMALL_TRIANGLE.mirror().setHeading(Math.PI / 2.0);


    public static double
            TIME_TO_SHOOT = 0.5,
            ANG_VELOCITY_MULTIPLER = 0.4,
            SLOW_MODE = 0.4;

    public static final int
            BLUE_GOAL_ID = 20,
            RED_GOAL_ID  = 24,
            MIN_DISTANCE_FEEDER = 0, // TODO mm
            MAX_DISTANCE_FEEDER = 100, // TODO mm
            MIN_SHOOTING_DISTANCE = 40,
            COLOR_SENSOR_UPDATE_LOOPS = (1 << 2) - 1;



    public static HSV
            GREEN_MIN_REV = new HSV(145, 0.5, 0.0),
            GREEN_MAX_REV = new HSV(160, 0.7, 2.0),
            PURPLE_MIN_REV = new HSV(180, 0.3, 0.0),
            PURPLE_MAX_REV = new HSV(230, 0.45, 2.0);

    public static HSV
            GREEN_MIN_CR = new HSV(55, 0.5, 0.0),
            GREEN_MAX_CR = new HSV(90, 0.77, 2.0),
            PURPLE_MIN_CR = new HSV(44.3, 0.45, 0.0),
            PURPLE_MAX_CR = new HSV(49.3, 0.77, 2.0),
            INTAKE_NONE_MIN_CR = new HSV(98, 0.275, 0),
            INTAKE_NONE_MAX_CR = new HSV(102, 0.415, 2);



    public static Robot robot;

    public static TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static MultipleTelemetry dashTelemetry;
}
