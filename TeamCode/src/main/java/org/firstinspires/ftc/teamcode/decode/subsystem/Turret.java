package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_ENCODER;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.TURRET_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Turret.TurretStates.ODOM_TRACKING;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.IIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.LowPassGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;
import org.firstinspires.ftc.teamcode.decode.util.CachedMotorEx;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;

@Configurable
public class Turret extends Subsystem<Turret.TurretStates> {
    private final CachedMotorEx turret;
    private final AnalogInput absoluteEncoder;
    private final Motor.Encoder motorEncoder;

    private final Follower dt;
    private final Shooter shooter;
    private final VoltageSensor voltageSensor;

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
        IDLE, ODOM_TRACKING, VISION_TRACKING
    }

    public static LowPassGains targetAngleGains = new LowPassGains(0.10);
    private final IIRLowPassFilter targetAngleFilter = new IIRLowPassFilter(targetAngleGains);

    private TurretStates currentState = ODOM_TRACKING;

    private final FIRLowPassFilter derivFilter = new FIRLowPassFilter(filterGains);
    private final PIDController controller = new PIDController(derivFilter);

    public static LowPassGains filterGains = new LowPassGains(0, 2);

    public static double
            kS = -0.1167,
            TICKS_TO_DEGREES = 90.0 / 148.0,
            WRAP_AROUND_ANGLE = 150,
            WRAP_AROUND_THRESHOLD = 5,
            THRESHOLD = 0.3,
            PID_SWITCH_ANGLE = 15,
            TOLERANCE_COUNTER = 10,
            PID_TOLERANCE = 3,
            MANUAL_POWER_MULTIPLIER = 0.7,
            ABSOLUTE_ENCODER_OFFSET = -30;

    public static int
            ZERO_TURRET_LOOPS = (1 << 5) - 1,
            CHECK_UNDETECTED_LOOPS = (1 << 5) - 1; // checking every X loops to switch to VISION_TRACKING state

    private Pose goal = Common.BLUE_GOAL;
    private Pose turretPos = new Pose(0, 0);

    private double
            currentAngle = 0.0,
            targetAngle = 0.0,
            targetAngleDebounced = 0.0,
            toleranceCounter = 0,
            encoderOffset = 0.0;

    public Turret(HardwareMap hw, Follower dt, VoltageSensor voltageSensor, Shooter shooter) {
        this.turret = new CachedMotorEx(hw, NAME_TURRET_MOTOR, Motor.GoBILDA.RPM_1150.getRPM(), THRESHOLD);
        MotorEx rightBack = new MotorEx(hw, "right back", Motor.GoBILDA.RPM_1150);
        absoluteEncoder = hw.get(AnalogInput.class, NAME_TURRET_ENCODER);
        this.turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorEncoder = rightBack.encoder;

        motorEncoder.reset();
        controller.setGains(closeGains);
        derivFilter.setGains(filterGains);

        this.dt = dt;
        this.voltageSensor = voltageSensor;
        this.shooter = shooter;
    }

    public void setGoalAlliance() {
        goal = Common.isRed ? Common.BLUE_GOAL.mirror() : Common.BLUE_GOAL;
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


    public void applyOffset() {
        encoderOffset = motorEncoder.getPosition() * TICKS_TO_DEGREES - normalizeToTurretRange((360 - ((absoluteEncoder.getVoltage() / 3.2 * 360 + ABSOLUTE_ENCODER_OFFSET) % 360)) % 360);
    }

    private void setTracking() {
        double rawKinAngleDeg = Math.toDegrees(shooter.kinematicsSolver.getTurretAngle());
        double normalized = ((rawKinAngleDeg % 360) + 360) % 360;
        double targetAngleRaw = normalizeToTurretRange(normalized);
        targetAngle = targetAngleFilter.calculate(targetAngleRaw);
        // Snap to the wrap boundary if within threshold to prevent wire overrun
        targetAngleDebounced = Math.abs(targetAngleRaw - WRAP_AROUND_ANGLE) < WRAP_AROUND_THRESHOLD
                ? WRAP_AROUND_ANGLE : targetAngle;
    }

    public boolean isPIDInTolerance() {
        return controller.isInTolerance(new State(currentAngle, 0, 0, 0), PID_TOLERANCE);
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

    // Inputs only from 0 - 360 degrees
    public static double normalizeToTurretRange(double angle) {
        return angle > WRAP_AROUND_ANGLE ? angle - 360 : angle;
    }

    @Override
    public void run() {
        currentAngle = (motorEncoder.getPosition() * TICKS_TO_DEGREES) - encoderOffset;

        double scalar = MAX_VOLTAGE / voltageSensor.getVoltage();

        controller.setGains(Math.abs(currentAngle - targetAngleDebounced) < PID_SWITCH_ANGLE ? closeGains : farGains);
        derivFilter.setGains(filterGains);
        targetAngleFilter.setGains(targetAngleGains);
        double robotHeading = dt.getHeading();
        turretPos = calculateTurretPosition(dt.getPose(), Math.toDegrees(robotHeading), -TURRET_OFFSET_Y);

        switch (currentState) {
            case IDLE:
                targetAngle = 0;
                targetAngleDebounced = 0;
                if (shooter.isBallPresent()) currentState = ODOM_TRACKING;
                break;
            case ODOM_TRACKING:
                setTracking();
        }

            boolean pidInTolerance = controller.isInTolerance(
                    new State(currentAngle, 0, 0, 0), 0.2
            );

            if (pidInTolerance) toleranceCounter++;
            else toleranceCounter = 0;

            if (((LoopUtil.getLoops() & ZERO_TURRET_LOOPS) == 0) && toleranceCounter >= TOLERANCE_COUNTER) applyOffset();

            double error = currentAngle - targetAngleDebounced;
            double ksOutput = error > PID_TOLERANCE ? kS * scalar : (error < -PID_TOLERANCE ? -kS * scalar : 0);
            controller.setTarget(new State(targetAngleDebounced, 0, 0, 0));
            double output = controller.calculate(new State(currentAngle, 0, 0, 0)) + ksOutput;

            turret.set(output);

        if (isPIDInTolerance() && shooter.getQueuedShots() <= 0) controller.reset();
    }

    public void printTelemetry() {
        turret.threshold = THRESHOLD;

        telemetry.addLine("TURRET");
        telemetry.addData("current state (ENUM): ", currentState);
        telemetry.addData("calculated distance (INCHES): ", getDistance());
        telemetry.addData("is PID in tolerance (BOOLEAN): ", isPIDInTolerance());

        dashTelemetry.addLine("TURRET");
        dashTelemetry.addData("vision setpoint (RADIANS): ", 0);
        dashTelemetry.addData("encoder angle (ANGLE): ", currentAngle);
        dashTelemetry.addData("raw motor ticks (TICKS): ", motorEncoder.getPosition());
        dashTelemetry.addData("absolute encoder (ANGLE): ", normalizeToTurretRange(360 - ((absoluteEncoder.getVoltage() / 3.2 * 360 + ABSOLUTE_ENCODER_OFFSET) % 360) % 360));
        dashTelemetry.addData("target angle (ANGLE): ", targetAngle);

        dashTelemetry.addLine("TURRET POSE (VISION/ODO)");
        dashTelemetry.addData("TURRET X (INCHES)", "%.4f", turretPos.getX());
        dashTelemetry.addData("TURRET Y (INCHES)", "%.4f", turretPos.getY());
        dashTelemetry.addData("Heading (DEGREES)", "%.1f", turretPos.getHeading());
    }
}
