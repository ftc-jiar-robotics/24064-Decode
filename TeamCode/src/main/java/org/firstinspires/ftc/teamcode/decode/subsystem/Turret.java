package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_ENCODER;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFuturePoseOn;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isRed;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Turret.TurretStates.ODOM_TRACKING;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.IIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.LowPassGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.Differentiator;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;
import org.firstinspires.ftc.teamcode.decode.util.CachedMotor;

@Configurable
public class Turret extends Subsystem<Turret.TurretStates> {
    private final CachedMotor turret;
    private final AnalogInput absoluteEncoder;
    private final Motor.Encoder motorEncoder;
//    private final AutoAim autoAim;
    public static PIDGains closeGains = new PIDGains(
            0.0196,
            0.0001,
            0.0002,
            Double.POSITIVE_INFINITY
    );

    public static PIDGains farGains = new PIDGains(
            0.0067,
            0,
            0.0000667,
            Double.POSITIVE_INFINITY
    );

    public enum TurretStates {
        IDLE, ODOM_TRACKING
    }

    private TurretStates currentState = TurretStates.IDLE;

    private final Differentiator differentiator = new Differentiator();
    public static LowPassGains targetAngleGains = new LowPassGains(0.10);
    private final IIRLowPassFilter targetAngleFilter = new IIRLowPassFilter(targetAngleGains);
    private final FIRLowPassFilter errorDerivFilter = new FIRLowPassFilter(errorDerivGains);
    private final PIDController controller = new PIDController(errorDerivFilter);
    public static LowPassGains errorDerivGains = new LowPassGains(0, 2);

    public static double
            kS = -0.07,
            LAUNCH_DELAY = 0.5,    // seconds (feeder > ball leaves flywheel) NOTE: 1 second at 11v, .7 at 12.3
            WRAP_AROUND_THRESHOLD = 5,
            SWITCH_Y_POSITION_BIG = 100,
            SWITCH_Y_POSITION_SMALL = 48,
            GOAL_ADDITION_X_BLUE = 4,
            GOAL_ADDITION_X_RED = 7,
            GOAL_SUBTRACTION_Y = 6,
            TICKS_TO_DEGREES = 0.232737218162581,
            WRAP_AROUND_ANGLE = 180,
            ROUNDING_POINT = 100000,
            PID_SWITCH_ANGLE = 15,
            PID_DERIV_TOLERANCE = 15,
            PID_TOLERANCE_CLOSE = 2,
            PID_TOLERANCE_FAR = 2,
            STATIC_TOLERANCE_SCALE = 1.0,   // when robot is basically still
            MOVING_TOLERANCE_SCALE = 1.8,   // when robot is moving (tune this)
            MANUAL_POWER_MULTIPLIER = 0.7,
            BADGE_RETRACTOR_ANGLE = 90,
            BADGE_RETRACTOR_KS = -0.15,
            ABSOLUTE_ENCODER_OFFSET = -177.4444,
            READY_TO_SHOOT_LOOPS = 3,
            OUT_OF_TOLERANCE_LOOPS = 3,
            kA_TURRET = 0,
            kV_TURRET = 0.1,   // start at 0, tune up slowly
            LOS_EPS = 1e-6;    // divide by zero guard

    private Pose goal = Common.BLUE_GOAL;
    private Pose turretPos = new Pose(0, 0);

    private double
            currentAngle = 0.0,
            targetAngle = 0.0,
            notInToleranceCounter = 0,
            toleranceCounter = 0,
            encoderOffset = 0.0,
            robotHeadingTurretDomain = 0.0,
            manualPower = 0.0,
            quadratureTurretAngle = 0.0;

    public Turret(HardwareMap hw) {
        this.turret = new CachedMotor(hw, NAME_TURRET_MOTOR, Motor.GoBILDA.RPM_435, ROUNDING_POINT);
        MotorEx rightBack = new MotorEx(hw, "right back", Motor.GoBILDA.RPM_435);
        absoluteEncoder = hw.get(AnalogInput.class, NAME_TURRET_ENCODER);
        this.turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        autoAim = new AutoAim(hw, NAME_TURRET_CAMERA);
        motorEncoder = rightBack.encoder;

        motorEncoder.reset();
        controller.setGains(closeGains);
        errorDerivFilter.setGains(errorDerivGains);
    }

//    public void closeAutoAim() {
//        autoAim.close();
//    }

    public double getError() {
        return controller.getError();
    }

    public Pose getGoal() {
        return goal;
    }

    private Pose setGoal() {
        Pose newGoal;

        double x = Common.BLUE_GOAL.getX();
        double y = Common.BLUE_GOAL.getY();
        if (!robot.isAuto && robot.drivetrain.getPose().getY() > SWITCH_Y_POSITION_BIG) newGoal = new Pose(x, y - GOAL_SUBTRACTION_Y);
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

    public double getDistance(Pose turretPos) {
        double dx = goal.getX() - turretPos.getX();
        double dy = goal.getY() - turretPos.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double getDistance() {
        return getDistance(turretPos);
    }

    double getCurrentAngle() {
        return currentAngle;
    }
    private double getPositionTolerance() {
        // Distance-based base tolerance
        double baseTol = getDistance() > 120 ? PID_TOLERANCE_FAR : PID_TOLERANCE_CLOSE;

        // Scale based on if the robot is moving
        boolean moving = robot.isRobotMoving();
        double scale = moving ? MOVING_TOLERANCE_SCALE : STATIC_TOLERANCE_SCALE;

        return baseTol * scale;
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

    void applyOffset() {
        applyOffset(true);
    }
    void applyOffset(boolean useAbs) {
        if (Common.TURRET_ENC_OFFSET == Double.POSITIVE_INFINITY || useAbs)
            encoderOffset = motorEncoder.getPosition() * TICKS_TO_DEGREES - getAbsoluteEncoderAngle();
        else encoderOffset = motorEncoder.getPosition() * TICKS_TO_DEGREES - Common.TURRET_ENC_OFFSET;
    }

    void setTracking() {
        setTracking(0);
    }

    void setTracking(double offsetDegrees) {
        double theta = calculateAngleToGoal(turretPos) + offsetDegrees;
        double alpha = ((theta - robotHeadingTurretDomain) + 3600) % 360;
        targetAngle = normalizeToTurretRange(alpha);
        double targetAngleRaw = targetAngle;
        targetAngle = targetAngleFilter.calculate(targetAngle);

        controller.setTarget(new State(Math.abs(targetAngleRaw - WRAP_AROUND_ANGLE) < WRAP_AROUND_THRESHOLD ? WRAP_AROUND_ANGLE : targetAngle, 0, 0, 0));
    }

    public boolean isPIDInTolerance() {
        double posTol = getPositionTolerance();
        return Math.abs(currentAngle - targetAngle) < posTol && controller.isInTolerance(new State(currentAngle, 0, 0, 0), posTol, PID_DERIV_TOLERANCE);
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
        return angle > WRAP_AROUND_ANGLE ? angle - 360 : angle;
    }

    // RAW abs encoder math
    public double getAbsoluteEncoderAngle() {
        double voltage = absoluteEncoder.getVoltage();

        double rawDegrees = (360 - (voltage / 3.24 * 360.0 + ABSOLUTE_ENCODER_OFFSET)) % 360.0;
        double turretDomain = (360.0 - rawDegrees) % 360.0;

        return normalizeToTurretRange(turretDomain);
    }

    public void setManual(double power) {
        manualPower = power * MANUAL_POWER_MULTIPLIER;
    }

    @Override
    public void run() {
        goal = setGoal();

        quadratureTurretAngle = (motorEncoder.getPosition() * TICKS_TO_DEGREES) - encoderOffset;
        currentAngle = quadratureTurretAngle;

        double error = currentAngle - targetAngle;

        PIDGains gains = Math.abs(error) < PID_SWITCH_ANGLE ? closeGains : farGains;

        double scalar = MAX_VOLTAGE / robot.batteryVoltageSensor.getVoltage();
        double kS = currentAngle > BADGE_RETRACTOR_ANGLE ? BADGE_RETRACTOR_KS : Turret.kS;
        double errorThreshold = robot.isRobotMoving() ? 0 : .1;
        double output = error > errorThreshold ? kS * scalar : (error < -errorThreshold ? -kS * scalar : 0);

        controller.setGains(gains);
        errorDerivFilter.setGains(errorDerivGains);
        targetAngleFilter.setGains(targetAngleGains);
        // turning robot heading to turret heading
        double robotHeading = robot.drivetrain.getHeading();
        robotHeadingTurretDomain = ((360 - Math.toDegrees(robotHeading)) + 90 + 3600) % 360;

        if (Math.abs(manualPower) > 0) turret.set(manualPower);

        else {
            switch (currentState) {
                case IDLE:
                    targetAngle = 0;
                    controller.setTarget(new State(targetAngle, 0, 0, 0));
                    differentiator.reset();
                    if (isPIDInTolerance()) applyOffset(true);
                    if (robot.shooter.isBallPresent()) currentState = ODOM_TRACKING;
                    break;
                case ODOM_TRACKING:
                    turretPos = calculateTurretPosition(robot.drivetrain.getPose(), Math.toDegrees(robotHeading), -Common.TURRET_OFFSET_Y);
//                    setTracking();

                    double alphaDot = getDesiredTurretOmegaRadPerSec();
                    output += (kA_TURRET * differentiator.getDerivative(alphaDot)) * scalar;
                    output += (kV_TURRET * alphaDot) * scalar;

                    break;
            }

            output += controller.calculate(new State(currentAngle, 0, 0 ,0));
            // LOS angular-rate feedforward (rad/s -> motor power

            if (isPIDInTolerance()) {
                toleranceCounter++;
                notInToleranceCounter = 0;
            } else {
                toleranceCounter = 0;
                notInToleranceCounter++;
            }

            turret.set(output);
        }


        if (isPIDInTolerance() && robot.shooter.getQueuedShots() <= 0) controller.reset();
    }

    public boolean isReadyToShoot() {
        // We already increment toleranceCounter only when we're in a very tight tolerance
        // So this is basically: "have we been super in-tolerance for a few loops in a row?"
        return toleranceCounter >= READY_TO_SHOOT_LOOPS;
    }

    public boolean isNotStable() {
        return notInToleranceCounter >= OUT_OF_TOLERANCE_LOOPS;
    }

    public void printTelemetry() {
        turret.setRoundingPoint(ROUNDING_POINT);

        telemetry.addLine("TURRET");
        telemetry.addData("current state (ENUM): ", currentState);
        telemetry.addData("calculated distance (INCHES): ", getDistance());
        telemetry.addData("is PID in tolerance (BOOLEAN): ", isPIDInTolerance());

        dashTelemetry.addLine("TURRET");
        dashTelemetry.addData("vision setpoint (RADIANS): ", 0);
//        dashTelemetry.addData("current vision (RADIANS): ", autoAim.getTargetYawDegrees());
        dashTelemetry.addData("encoder angle (ANGLE): ", currentAngle);
        dashTelemetry.addData("raw motor ticks (TICKS): ", motorEncoder.getPosition());
        dashTelemetry.addData("absolute encoder (ANGLE): ", getAbsoluteEncoderAngle());
        dashTelemetry.addData("absolute encoder (VOLTAGE): ", absoluteEncoder.getVoltage());
        dashTelemetry.addData("target angle (ANGLE): ", targetAngle);
        dashTelemetry.addData("quadrature turret angle (ANGLE): ", quadratureTurretAngle);

        dashTelemetry.addLine("TURRET POSE (VISION/ODO)");
        dashTelemetry.addData("TURRET X (INCHES)", "%.4f", turretPos.getX());
        dashTelemetry.addData("TURRET Y (INCHES)", "%.4f", turretPos.getY());
        dashTelemetry.addData("Heading (DEGREES)", "%.1f", turretPos.getHeading());
    }
}
