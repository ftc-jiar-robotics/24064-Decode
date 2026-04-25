package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_ENCODER;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_MASTER_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_SLAVE_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.SERVO_AXON_MIN;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.SERVO_AXON_MINI_MK2_MAX;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFuturePoseOn;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isRed;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Turret.TurretStates.ODOM_TRACKING;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.IIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.LowPassGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.Differentiator;
import org.firstinspires.ftc.teamcode.decode.util.CachedServo;

@Configurable
public class Turret extends Subsystem<Turret.TurretStates> {
    private final CachedServo turretMaster;
    private final CachedServo turretSlave;
    private final AnalogInput absoluteEncoder;

    public enum TurretStates {
        IDLE, ODOM_TRACKING
    }

    private Differentiator radialVelocity = new Differentiator(); // differentiates radial vel of turret to compensate for deaccel in sotm
    private TurretStates currentState = TurretStates.IDLE;

    private final Differentiator differentiator = new Differentiator();
    public static LowPassGains targetAngleGains = new LowPassGains(0);
    public static LowPassGains accelerationGains = new LowPassGains(0.9);
    private final IIRLowPassFilter targetAngleFilter = new IIRLowPassFilter(targetAngleGains);

    private final FIRLowPassFilter accelerationFilter = new FIRLowPassFilter(accelerationGains);

    public static double
            ACCEL_TOLERANCE = 100,
            OFFSET_MULTIPLER = 1,
            WRAP_AROUND_THRESHOLD = 5,
            READY_TO_SHOOT_LOOPS = 3,
            SWITCH_Y_POSITION_BIG = 100,
            SWITCH_Y_POSITION_SMALL = 48,
            GOAL_ADDITION_X_BLUE = 4,
            GOAL_ADDITION_X_RED = 7,
            GOAL_SUBTRACTION_Y = 2,
            WRAP_AROUND_ANGLE = 180,
            TURRET_CLIP_ANGLE_MIN = 40,
            TURRET_CLIP_ANGLE_MAX = 340,
            ANGLE_TOLERANCE = 3,
            STATIC_TOLERANCE_SCALE = 1.0,   // when robot is basically still
            MOVING_TOLERANCE_SCALE = 1.8,   // when robot is moving (tune this)
            ABSOLUTE_ENCODER_OFFSET = -96.4444,
            LAUNCH_DELAY = 0.3,
            LOS_EPS = 1e-6,
            ROUNDING_POINT = 20;    // divide by zero guard

    private Pose goal = Common.BLUE_GOAL;

    private Pose turretPos = new Pose(0, 0);

    private double
            radialAcceleration = 0.0,
//            currentAngle = 0.0,
            targetAngle = 0.0,
            targetAngleDebounced = 0.0,
            toleranceCounter = 0,
            robotHeadingTurretDomain = 0.0;

    public Turret(HardwareMap hw) {
        this.turretMaster = new CachedServo(hw, NAME_TURRET_MASTER_SERVO, SERVO_AXON_MIN, SERVO_AXON_MINI_MK2_MAX, AngleUnit.DEGREES, ROUNDING_POINT);
        this.turretSlave = new CachedServo(hw, NAME_TURRET_SLAVE_SERVO, SERVO_AXON_MIN, SERVO_AXON_MINI_MK2_MAX, AngleUnit.DEGREES, ROUNDING_POINT);

        absoluteEncoder = hw.get(AnalogInput.class, NAME_TURRET_ENCODER);

        turretMaster.setPwmRange(500, 2500);
        turretSlave.setPwmRange(500, 2500);
        turretMaster.setInverted(true);
        turretSlave.setInverted(true);

    }


    private Pose setGoal() {
        Pose newGoal;

        double x = Common.BLUE_GOAL.getX();
        double y = Common.BLUE_GOAL.getY();
        if (robot.drivetrain.getPose().getY() > SWITCH_Y_POSITION_BIG) newGoal = new Pose(x, y - GOAL_SUBTRACTION_Y);
        else if (robot.isAuto || robot.drivetrain.getPose().getY() < SWITCH_Y_POSITION_SMALL) newGoal = new Pose(x + (isRed ? GOAL_ADDITION_X_RED : GOAL_ADDITION_X_BLUE), y);
        else newGoal = new Pose(x, y);


        return isRed ? newGoal.mirror() : newGoal;
    }

    public void setAlliance() {
        goal = Common.isRed ? Common.BLUE_GOAL.mirror() : Common.BLUE_GOAL;
//        autoAim.setAlliance();
    }

    @Override
    public void set(TurretStates a) {
        currentState = a;
    }

    @Override
    public TurretStates get() {
        return currentState;
    }

    public Pose getTurretPos() {
        return turretPos;
    }

    public double getDistance(Pose turretPos) {
        double dx = goal.getX() - turretPos.getX();
        double dy = goal.getY() - turretPos.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double getDistance() {
        return getDistance(turretPos);
    }

//    double getCurrentAngle() {
//        return currentAngle;
//    }
    private double getPositionTolerance() {
        // Scale based on if the robot is moving
        boolean moving = robot.isRobotMoving();
        double scale = moving ? MOVING_TOLERANCE_SCALE : STATIC_TOLERANCE_SCALE;

        return 1 * scale;
    }

    private double getDesiredTurretOmegaRadPerSec() {
        // Vector from turret to goal (field)
        double dx = goal.getX() - turretPos.getX();
        double dy = goal.getY() - turretPos.getY();

        double denom = dx * dx + dy * dy;
        if (denom < LOS_EPS) return 0.0;

        double vx = robot.drivetrain.getVelocity().getXComponent();
        double vy = robot.drivetrain.getVelocity().getYComponent();

        // Robot angular velocity (rad/s)
        double omega = robot.drivetrain.getAngularVelocity();

        // LOS rate for atan2(dy, dx) in rad/s
        double phiDot = (dy * vx - dx * vy) / denom;

        // angle domain flips sign (360 - angle), and heading domain also flips,
        // so the relative turret domain rate becomes: alphaDot = omega - phiDot
        return omega - phiDot;
    }

    private void setTracking() {
        setTracking(turretPos);
    }

    private void setTracking(Pose customTurretPos) {
        double theta = calculateAngleToGoal(customTurretPos);
        double alpha = ((theta - robotHeadingTurretDomain) + 3600) % 360;
        turretPos.setHeading(robot.drivetrain.getHeading()-alpha);

        alpha -= Math.toDegrees(robot.shooter.getCompensatedValues()[2]) * (accelerationFilter.calculate(Math.abs(robot.drivetrain.getAcceleration().getMagnitude())) < ACCEL_TOLERANCE ? OFFSET_MULTIPLER : 0);
        targetAngle = normalizeToTurretRange(alpha);
        double targetAngleRaw = targetAngle;
        targetAngle = targetAngleFilter.calculate(targetAngle);
        targetAngleDebounced = Math.abs(targetAngleRaw - WRAP_AROUND_ANGLE) < WRAP_AROUND_THRESHOLD ? WRAP_AROUND_ANGLE : targetAngle;
    }

    public boolean isPIDInTolerance() {
        return Math.abs(targetAngle - getAbsoluteEncoderAngle()) < ANGLE_TOLERANCE;
    }


    /**
     * Calculate the turret position (xt, yt).
     * Formula: xt = x - cos(heading) * D, yt = y - sin(heading) * D
     */
    public static Pose calculateTurretPosition(Pose robotPos, double headingDeg, double offset) {
        double headingRad = Math.toRadians(headingDeg);

        double xOffset = Math.cos(headingRad) * offset;
        double yOffset = Math.sin(headingRad) * offset;

        double xt = robotPos.getX() - xOffset;
        double yt = robotPos.getY() - yOffset;

        return new Pose(xt, yt);
    }

    /**
     * Calculate θ, the raw angle from turret to goal.
     * Formula: atan2(yg - yt, xg - xt)
     */
    public double calculateAngleToGoal(Pose turretPos) {
        double dx = goal.getX() - turretPos.getX();
        double dy = goal.getY() - turretPos.getY();
        
        return ((360 - Math.toDegrees(Math.atan2(dy, dx))) + 90 + 3600) % 360;
    }

    // Inputs only from 0 - 360 degrees
    public static double normalizeToTurretRange(double angle) {
        return (WRAP_AROUND_ANGLE-angle +3600)%360;//angle > WRAP_AROUND_ANGLE ? angle - 360 : angle;
    }

    // RAW abs encoder math
    public double getAbsoluteEncoderAngle() {
        double voltage = absoluteEncoder.getVoltage();

        double rawDegrees = (360 - (voltage / 3.24 * 360.0 + ABSOLUTE_ENCODER_OFFSET)) % 360.0;
//        double turretDomain = (360.0 - rawDegrees) % 360.0;

        return normalizeToTurretRange(rawDegrees);
    }

    @Override
    public void run() {
        goal = setGoal();
        accelerationFilter.setGains(accelerationGains);
        targetAngleFilter.setGains(targetAngleGains);

        // turning robot heading to turret heading
        double robotHeading = robot.drivetrain.getHeading();
        robotHeadingTurretDomain = ((360 - Math.toDegrees(robotHeading)) + 90 + 3600) % 360;
        turretPos = calculateTurretPosition(robot.drivetrain.getPose(), Math.toDegrees(robotHeading), -Common.TURRET_OFFSET_Y);

        switch (currentState) {
            case IDLE:
//                targetAngle = 180;
                setTracking();

//                setTracking(calculateTurretPosition(LaunchZone.getInterceptOrClosestPoint(), Math.toDegrees(robotHeading), -Common.TURRET_OFFSET_Y));
                differentiator.reset();
                if (robot.shooter.isBallPresent()) currentState = ODOM_TRACKING;
                break;
            case ODOM_TRACKING:
                setTracking();
                break;
        }

        if (isPIDInTolerance()) toleranceCounter++;
        else toleranceCounter = 0;

        double targetAngleClipped = Range.clip(targetAngleDebounced, TURRET_CLIP_ANGLE_MIN, TURRET_CLIP_ANGLE_MAX);

        turretMaster.turnToAngle(targetAngleClipped);
        turretSlave.turnToAngle(targetAngleClipped);
    }

    public boolean isReadyToShoot() {
        // We already increment toleranceCounter only when we're in a very tight tolerance
        // So this is basically: "have we been super in-tolerance for a few loops in a row?"
        return toleranceCounter >= READY_TO_SHOOT_LOOPS;
    }

    public void printTelemetry() {
        telemetry.addLine("TURRET");
        telemetry.addData("current state (ENUM): ", currentState);
        telemetry.addData("calculated distance (INCHES): ", getDistance());
        telemetry.addData("is PID in tolerance (BOOLEAN): ", isPIDInTolerance());

        dashTelemetry.addLine("TURRET");
        dashTelemetry.addData("vision setpoint (RADIANS): ", 0);
        dashTelemetry.addData("encoder angle (ANGLE): ", turretMaster.getAngle());
        dashTelemetry.addData("absolute encoder (ANGLE): ", getAbsoluteEncoderAngle());
        dashTelemetry.addData("absolute encoder (VOLTAGE): ", absoluteEncoder.getVoltage());
        dashTelemetry.addData("target angle (ANGLE): ", targetAngle);

        dashTelemetry.addLine("TURRET POSE (VISION/ODO)");
        dashTelemetry.addData("TURRET X (INCHES)", "%.4f", turretPos.getX());
        dashTelemetry.addData("TURRET Y (INCHES)", "%.4f", turretPos.getY());
        dashTelemetry.addData("Heading (DEGREES)", "%.1f", turretPos.getHeading());
    }
}
