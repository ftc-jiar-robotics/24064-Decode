package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_ENCODER;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFuturePoseOn;
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
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.Filter;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.LowPassGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.MovingAverageGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;
import org.firstinspires.ftc.teamcode.decode.util.CachedMotor;

@Configurable
public class Turret extends Subsystem<Turret.TurretStates> {
    private final CachedMotor turret;
    private final AnalogInput absoluteEncoder;
    private final Motor.Encoder motorEncoder;
//    private final AutoAim autoAim;
    public static PIDGains closeGains = new PIDGains(
            0.01925,
            0,
            0.00025,
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

    private TurretStates currentState = ODOM_TRACKING;

    public static MovingAverageGains targetAngleAverageGains = new MovingAverageGains(
            6
    );

    private final Filter targetAngleAverageFilter = new MovingAverageFilter(targetAngleAverageGains);

    private final FIRLowPassFilter derivFilter = new FIRLowPassFilter(filterGains);
    private final PIDController controller = new PIDController(derivFilter);
    public static LowPassGains filterGains = new LowPassGains(0, 2);

    private double[] visionVariances = new double[3];

    public static double
            kS = -0.1167,
            BADGE_RETRACTOR_kS = -0.3,
            BADGE_RETRACTOR_SWITCH_ANGLE = 10,
            TICKS_TO_DEGREES = 0.63,
            WRAP_AROUND_ANGLE = 150,
            ROUNDING_POINT = 100000,
            PID_SWITCH_ANGLE = 15,
            PID_TOLERANCE = 3,
            DERIV_TOLERANCE = 4,
            MANUAL_POWER_MULTIPLIER = 0.7,
            ABSOLUTE_ENCODER_OFFSET = -298.575,
            READY_TO_SHOOT_LOOPS = 3;



    private Pose goal = Common.BLUE_GOAL;
    private Pose turretPos = new Pose(0, 0);
    private Pose robotPoseFromVision = new Pose(0, 0);

    private double
            currentAngle = 0.0,
            targetAngle = 0.0,
            toleranceCounter = 0,
            encoderOffset = 0.0,
            robotHeadingTurretDomain = 0.0,
            rawPower = 0.0,
            manualPower = 0.0,
            quadratureTurretAngle = 0.0;
    private boolean isOffsetCalibrating = false;
    private int offsetSamplesTaken = 0;
    private double offsetAngleSum = 0.0;

    public Turret(HardwareMap hw) {
        this.turret = new CachedMotor(hw, NAME_TURRET_MOTOR, Motor.GoBILDA.RPM_1150, ROUNDING_POINT);
        MotorEx rightBack = new MotorEx(hw, "right back", Motor.GoBILDA.RPM_1150);
        absoluteEncoder = hw.get(AnalogInput.class, NAME_TURRET_ENCODER);
        this.turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        autoAim = new AutoAim(hw, NAME_TURRET_CAMERA);
        motorEncoder = rightBack.encoder;

        motorEncoder.reset();
        controller.setGains(closeGains);
        derivFilter.setGains(filterGains);
    }

//    public void closeAutoAim() {
//        autoAim.close();
//    }

    public void setGoalAlliance() {
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

    public double getDistance() {
        double dx = goal.getX() - turretPos.getX();
        double dy = goal.getY() - turretPos.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    double getCurrentAngle() {
        return currentAngle;
    }

    void applyOffset() {
        applyOffset(false);
    }
    void applyOffset(boolean useAbs) {
        if (Common.TURRET_ENC_OFFSET == Double.POSITIVE_INFINITY || useAbs)
            encoderOffset = motorEncoder.getPosition() * TICKS_TO_DEGREES - getAbsoluteEncoderAngle();
        else encoderOffset = motorEncoder.getPosition() * TICKS_TO_DEGREES - Common.TURRET_ENC_OFFSET;
    }

    private void setTracking() {
        double theta = calculateAngleToGoal(turretPos);
        double alpha = ((theta - robotHeadingTurretDomain) + 3600) % 360;
        targetAngle = normalizeToTurretRange(alpha);
        targetAngle = targetAngleAverageFilter.calculate(targetAngle);

        controller.setTarget(new State(targetAngle, 0, 0, 0));
    }

    public boolean isPIDInTolerance() {
        return controller.isInTolerance(new State(currentAngle, 0, 0, 0), PID_TOLERANCE, DERIV_TOLERANCE);
    }

    /**
     * Calculate the turret position (xt, yt).
     * Formula: xt = x - cos(heading) * D, yt = y - sin(heading) * D
     */
    public Pose calculateTurretPosition(Pose robotPos, double headingDeg, double offset) {
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

        double rawDegrees = (voltage / 3.2 * 360.0 + ABSOLUTE_ENCODER_OFFSET) % 360.0;
        double turretDomain = (360.0 - rawDegrees) % 360.0;

        return normalizeToTurretRange(turretDomain);
    }

    public void setManual(double power) {
        manualPower = power * MANUAL_POWER_MULTIPLIER;
    }

    @Override
    public void run() {
        quadratureTurretAngle = (motorEncoder.getPosition() * TICKS_TO_DEGREES) - encoderOffset;
        currentAngle = quadratureTurretAngle;
        // Use quadrature to detect wraparound region
//        boolean inWraparoundZone = quadratureTurretAngle > WRAP_AROUND_ANGLE || quadratureTurretAngle < WRAP_AROUND_ANGLE - 360;

        // In wrap -> trust quadrature; elsewhere -> trust filtered abs encoder
//        currentAngle = inWraparoundZone ? quadratureTurretAngle : getAbsoluteEncoderAngle();

        double error = currentAngle - targetAngle;

        PIDGains gains = Math.abs(error) < PID_SWITCH_ANGLE ? closeGains : farGains;

        double scalar = MAX_VOLTAGE / robot.batteryVoltageSensor.getVoltage();
        double kStatic = currentAngle > BADGE_RETRACTOR_SWITCH_ANGLE ? BADGE_RETRACTOR_kS : kS;
        double output = error > PID_TOLERANCE ? kS * scalar : (error < -PID_TOLERANCE ? -kStatic * scalar : 0);

        controller.setGains(gains);
        derivFilter.setGains(filterGains);
        // turning robot heading to turret heading
        double robotHeading = isFuturePoseOn ? robot.shooter.getPredictedPose().getHeading() : robot.drivetrain.getHeading();
        robotHeadingTurretDomain = ((360 - Math.toDegrees(robotHeading)) + 90 + 3600) % 360;

        if (Math.abs(manualPower) > 0) turret.set(manualPower);

        else {
            switch (currentState) {
                case IDLE:
                    targetAngle = 0;
                    controller.setTarget(new State(targetAngle, 0, 0, 0));
                    if (isReadyToShoot()) applyOffset(true);
                    if (robot.shooter.isBallPresent()) currentState = ODOM_TRACKING;
                    break;
                case ODOM_TRACKING:
                    turretPos = calculateTurretPosition(isFuturePoseOn ? robot.shooter.getPredictedPose() : robot.drivetrain.getPose(), Math.toDegrees(robotHeading), -Common.TURRET_OFFSET_Y);
                    setTracking();
                    break;
            }

            output += controller.calculate(new State(currentAngle, 0, 0 ,0));
            rawPower = output;

            if (isPIDInTolerance()) {
                toleranceCounter++;
                output = 0;
            } else toleranceCounter = 0;


            turret.set(output);
        }

        if (isPIDInTolerance() && robot.shooter.getQueuedShots() <= 0) controller.reset();
    }

    public boolean isReadyToShoot() {
        // We already increment toleranceCounter only when we're in a very tight tolerance
        // So this is basically: "have we been super in-tolerance for a few loops in a row?"
        return toleranceCounter >= READY_TO_SHOOT_LOOPS;
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
        dashTelemetry.addData("target angle (ANGLE): ", targetAngle);
        dashTelemetry.addData("quadrature turret angle (ANGLE): ", quadratureTurretAngle);

        dashTelemetry.addLine("TURRET POSE (VISION/ODO)");
        dashTelemetry.addData("TURRET X (INCHES)", "%.4f", turretPos.getX());
        dashTelemetry.addData("TURRET Y (INCHES)", "%.4f", turretPos.getY());
        dashTelemetry.addData("Heading (DEGREES)", "%.1f", turretPos.getHeading());
    }
}
