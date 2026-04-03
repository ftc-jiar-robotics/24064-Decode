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
            NAME_FEEDER_BACK1_SERVO = "feederBack1",
            NAME_FEEDER_BACK2_SERVO = "feederBack2",
            NAME_FEEDER_GATE_SERVO = "feederGate",
            NAME_FEEDER_COLOR_SENSOR = "colorSensorFeeder",
            NAME_GATE_OPENER_RED_SERVO = "gateOpenerRed",
            NAME_GATE_OPENER_BLUE_SERVO = "gateOpenerBlue",
            NAME_FEEDER_LEFT_DISTANCE_SENSOR = "leftDistanceSensorFeeder",
            NAME_FEEDER_RIGHT_DISTANCE_SENSOR = "rightDistanceSensorFeeder",
            NAME_FEEDER_LEFT_PIN0 = "leftDistancePin0",
            NAME_FEEDER_RIGHT_PIN0 = "rightDistancePin0",
            NAME_INTAKE_FRONT_PIN0 = "intakeSensorFront",
            NAME_INTAKE_BACK_PIN0 = "intakeSensorBack",

            NAME_FLYWHEEL_MASTER_MOTOR = "shooterMaster",
            NAME_FLYWHEEL_SLAVE_MOTOR = "shooterSlave",

            NAME_HOOD_SERVO = "hood",

            NAME_INTAKE_MOTOR = "intake",
            NAME_INTAKE_MOTO_MOTOR = "intakeFeederMotor",
            NAME_INTAKE_DISTANCE_SENSOR = "distanceSensorIntake",

            NAME_TURRET_MASTER_SERVO = "turretMaster",
            NAME_TURRET_SLAVE_SERVO = "turretSlave",
            NAME_TURRET_ENCODER = "turretEncoder",
            NAME_TURRET_CAMERA = "arduCam",

            NAME_PTO_SERVO = "pto",
            NAME_BRAKE_MASTER_SERVO = "brakeMaster",
            NAME_BRAKE_SLAVE_SERVO = "brakeSlave";

    public static Pose AUTO_END_POSE = null;

    public static boolean
            isFuturePoseOn = false,
            isSlowMode = false,
            isTelemetryOn = false,
            isForwardPower = false,
            isStrafePower = false,
            isRed = false,
            isHoodManual = false,
            isFlywheelManual = false,
            inTriangle = false;

    public static final double
            INCHES_PER_METER = 39.3701,
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
            SERVO_AXON_MINI_MK2_MAX = 360,

            FORWARD_POD_Y = 84, // 84 mm
            STRAFE_POD_X = -152, //

            // Camera Stuff
            CAM_HEIGHT = 9.4375,
            CAM_PITCH = 0,

            // Camera offset from turret center (inches) +X = right, -X = left | +Y = forward, -Y = backward
            CAM_OFFSET_X = 0,
            CAM_OFFSET_Y = 7.5,
            TURRET_OFFSET_Y = -2.48592,

            MAX_VOLTAGE = 14;

    public static final Pose
            RED_BIG_TRIANGLE = new Pose(112.2, 135.5, Math.toRadians(270)),
            RED_SMALL_TRIANGLE = new Pose(88.125, 7.5, Math.toRadians(90)),
            BLUE_BIG_TRIANGLE = RED_BIG_TRIANGLE.mirror(),
            BLUE_SMALL_TRIANGLE = RED_SMALL_TRIANGLE.mirror().setHeading(Math.toRadians(90)),
            BLUE_GOAL = new Pose(0,144),
            RED_BLUE = BLUE_GOAL.mirror();

    public static double getAirtimeForDistance(double distanceInches) {
        double t = AIRTIME_A * distanceInches + AIRTIME_B;
        return Math.max(MIN_AIRTIME, t);
    }

    public static double
            MIN_MOVEMENT_SPEED = 0.5,
            ANG_VELOCITY_MULTIPLER = 0.15,
            FAR_DISTANCE = 120,
            MID_DISTANCE = 100,

            IMU_YAW_SCALAR = 1.00086,
            SLOW_MODE = 0.55,
            AIRTIME_A    = 0.0025,  // seconds per inch (tune) how much airtime increases per inch of distance.
            AIRTIME_B    = 0.03,    // base airtime (tune) minimum airtime when distance is zero.
            MIN_AIRTIME  = 0.02, //safety
            MIN_POWER_INPUT = 0.3,
            MAX_VELOCITY_MAGNITUDE = 0.2,
            LOCALIZATION_X = 10,
            LOCALIZATION_Y = 7.5,
            TURRET_ENC_OFFSET = Double.POSITIVE_INFINITY;
    public static final int
            BLUE_GOAL_ID = 20,
            RED_GOAL_ID  = 24,
            MIN_DISTANCE_FEEDER = 0, // TODO mm
            MAX_DISTANCE_FEEDER = 90, // TODO mm
            MIN_SHOOTING_DISTANCE = 49,
            RELOCALIZE_UPDATE_LOOPS = (1 << 3) - 1,
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
