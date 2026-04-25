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
            NAME_INTAKE_FRONT_PIN0 = "intakeSensorFront0",

            NAME_INTAKE_FRONT_PIN1= "intakeSensorFront1",
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
            NAME_TURRET_CAMERA = "arduCam";

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
            CAM_HEIGHT = 10,
            CAM_PITCH = 0,

            // Camera offset from turret center (inches) +X = right, -X = left | +Y = forward, -Y = backward
            CAM_OFFSET_X = 0,
            CAM_OFFSET_Y = 7,
            TURRET_OFFSET_Y = -2.48592,

            MAX_VOLTAGE = 14;

    public static final Pose
            RED_BIG_TRIANGLE = new Pose(112.2, 135.5, Math.toRadians(270)),
            RED_SMALL_TRIANGLE = new Pose(89.5, 7.125, Math.toRadians(90)),
            BLUE_BIG_TRIANGLE = new Pose(32,135.75,Math.toRadians(270)),
            BLUE_SMALL_TRIANGLE = RED_SMALL_TRIANGLE.mirror().setHeading(Math.toRadians(90));

    public static Pose
            RELOCALIZATION_GATE_RED = new Pose(144 - 14.875, 72 + 9.125, Math.toRadians(270)),
            BLUE_GOAL = new Pose(2,142),
            RED_GOAL = BLUE_GOAL.mirror();

    public static double
            MIN_MOVEMENT_SPEED = 0.5,
            ANG_VELOCITY_MULTIPLER = 0.15,
            FAR_DISTANCE = 110,
            MID_DISTANCE = 90,
            IMU_YAW_SCALAR = 1.001575,
            SLOW_MODE = 0.55,
            MIN_POWER_INPUT = 0.3,
            LOCALIZATION_X = 10,
            LOCALIZATION_Y = 7.5,
            TURRET_ENC_OFFSET = Double.POSITIVE_INFINITY;
    public static final int
            BLUE_GOAL_ID = 20,
            RED_GOAL_ID  = 24,
            MIN_DISTANCE_FEEDER = 0, // TODO mm
            MAX_DISTANCE_FEEDER = 80, // TODO mm
            COLOR_SENSOR_UPDATE_LOOPS = (1 << 2) - 1;

    public static Robot robot;

    public static TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static MultipleTelemetry dashTelemetry;
}
