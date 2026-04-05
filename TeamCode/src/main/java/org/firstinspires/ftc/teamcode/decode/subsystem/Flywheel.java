package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FLYWHEEL_MASTER_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FLYWHEEL_SLAVE_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFlywheelManual;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.decode.control.Ranges;
import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.SISOKalmanFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.KalmanGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;
import org.firstinspires.ftc.teamcode.decode.util.CachedMotorEx;

@Configurable
@Config
public class Flywheel extends Subsystem<Flywheel.FlyWheelStates> {

    private final CachedMotorEx[] motors;
    private final Motor.Encoder encoder;
    private final VoltageSensor batteryVoltageSensor;

    public enum FlyWheelStates {
        IDLE, ARMING, RUNNING
    }

    public static double
            FLYWHEEL_DIAMETER = 3, // INCHES
            MIN_MOVEMENT_SPEED = 35,
            LAUNCH_DELAY = 0.3,
            OUT_OF_TOLERANCE_LOOPS = 3,
            RPM_TOLERANCE = 30,
            LOW_PASS_FILTER_RPM_TOLERANCE = 250,
            RPM_TOLERANCE_WHILE_MOVING = 60,
            SMOOTH_RPM_GAIN = .7,
            DERIV_TOLERANCE = 200,
            IDLE_RPM = 1200,
            FAR_ARMING_RPM = 2950,
            CLOSE_ARMING_RPM = 2100,
            BB_TOLERANCE = 50000,
            MAX_RPM = 4000,
            TARGET_RPM_STEP = 30.0,
            TARGET_RPM_MID_BAND = 9.0,
            CACHE_THRESHOLD_MOTORS = 0.001,
            BATTERY_VOLTAGE_TUNED_AT = 13;

    private FlyWheelStates targetState = FlyWheelStates.IDLE;

    public static KalmanGains
            rpmFilterGains = new KalmanGains(1, 2),
            derivFilterGains = new KalmanGains(3, 0);
    private final SISOKalmanFilter
            rpmFilter = new SISOKalmanFilter(rpmFilterGains),
            derivFilter = new SISOKalmanFilter(derivFilterGains);

    public static PIDGains FLYWHEEL_PIDF_COEFFICIENTS = new PIDGains(0.0067, 0, 0, 1);
    private final PIDController velocityController = new PIDController(derivFilter);

    boolean ballIsPresent, movingToFarZone;

    private double rawRPM, manualPower, pidf;
    private final State shootingRPM = new State(4000,0,0,0);
    private final State currentRPMSmooth = new State();
    public double getCurrentRPMSmooth() {
        return currentRPMSmooth.x;
    }

    private static double quantizeWithMidpointBand(double rpmRaw, double step, double band) {
        double low = Math.floor(rpmRaw / step) * step;
        double high = low + step;
        double mid = (low + high) / 2.0;

        // If we're near the midpoint (like 2375), use midpoint so it doesn't flip bins
        if (abs(rpmRaw - mid) <= band) return mid;

        // Otherwise normal rounding to nearest step
        return Math.round(rpmRaw / step) * step;
    }

    public Flywheel(HardwareMap hw) {

        motors = new CachedMotorEx[]{
                new CachedMotorEx(hw, NAME_FLYWHEEL_MASTER_MOTOR, Motor.GoBILDA.BARE).reversed(),
                new CachedMotorEx(hw, NAME_FLYWHEEL_SLAVE_MOTOR, Motor.GoBILDA.BARE)
        };

        encoder = new MotorEx(hw, NAME_FLYWHEEL_MASTER_MOTOR, Motor.GoBILDA.BARE).encoder;
        encoder.setDirection(Motor.Direction.FORWARD);

        batteryVoltageSensor = hw.voltageSensor.iterator().next();
    }

    public void set(FlyWheelStates f) {
        targetState = f;
    }
    public FlyWheelStates get() {
        return targetState;
    }

    public void setManualPower(double power) {
        manualPower = power;
    }

    public boolean isPIDInTolerance() {
        return velocityController.isInTolerance(
                currentRPMSmooth,
                robot.isRobotMoving() ? RPM_TOLERANCE_WHILE_MOVING : RPM_TOLERANCE,
                DERIV_TOLERANCE
        );
    }

    @Override
    public void run() {

        rpmFilter.setGains(rpmFilterGains);
        derivFilter.setGains(derivFilterGains);
        velocityController.setGains(FLYWHEEL_PIDF_COEFFICIENTS);

        rawRPM = encoder.getCorrectedVelocity() * 60.0 / 28.0;
        currentRPMSmooth.x = rpmFilter.calculate(rawRPM);

        double voltageScalar = BATTERY_VOLTAGE_TUNED_AT / batteryVoltageSensor.getVoltage();

        if (rawRPM > 10000) rawRPM = 0;
        if (currentRPMSmooth.x > 10000) currentRPMSmooth.x = 0;

        switch (targetState) {
            case IDLE:
                if (!isFlywheelManual)
                    shootingRPM.x = !ballIsPresent ? IDLE_RPM :
                                    movingToFarZone ? FAR_ARMING_RPM : CLOSE_ARMING_RPM;
                break;
            case ARMING:
                if (isPIDInTolerance()) targetState = FlyWheelStates.RUNNING;
            case RUNNING:
                shootingRPM.x = quantizeWithMidpointBand(inchesPerSecondToRPM(robot.shooter.getCompensatedValues()[0]), TARGET_RPM_STEP, TARGET_RPM_MID_BAND);
                break;
        }

        velocityController.setTarget(shootingRPM);
        pidf = velocityController.calculate(currentRPMSmooth)
                + getFeedForward(shootingRPM.x) * voltageScalar;

        double output =
                        manualPower != 0 ? manualPower :
                        shootingRPM.x == 0 ? 0 :
                        shootingRPM.x - currentRPMSmooth.x > BB_TOLERANCE ? 1 :
                        Ranges.clip(pidf, 0, 1);

        for (CachedMotorEx m : motors) {
            m.threshold = CACHE_THRESHOLD_MOTORS;
            m.set(output);
        }

        if (isPIDInTolerance() && robot.shooter.getQueuedShots() <= 0)
            velocityController.reset();
    }

    private static double getFeedForward(double setpoint) {
        return 0;
    }

    public void incrementFlywheelRPM(double RPM) {
        shootingRPM.x += RPM;
        velocityController.setTarget(shootingRPM);
    }

    public static double inchesPerSecondToRPM(double x) {
        return (0.0571374 * x * x) + (-6.80299 * x) + 985.1611;
    }

    public static double RPMToInchesPerSecond(double x) {
        return (-0.0000175444 * x * x) + (0.13418 * x) + 8.4288;
    }

    private void chooseShootingRPM(double distance) {
        if (isFlywheelManual) return;
        double rpmRaw = 957.2952559300876*(1) + 14.312109862671662*(distance);
        shootingRPM.x = quantizeWithMidpointBand(rpmRaw, TARGET_RPM_STEP, TARGET_RPM_MID_BAND);
        velocityController.setTarget(shootingRPM);
    }


    public void printTelemetry() {
        telemetry.addLine("FLYWHEEL");
        telemetry.addData("current state (ENUM): ", get());
        telemetry.addData("target state (ENUM): ", targetState);
        telemetry.addData("current RPM (ROTATIONS PER MINUTE): ", rawRPM);
        telemetry.addData("is PID in tolerance (BOOLEAN): ", isPIDInTolerance());

        dashTelemetry.addLine("FLYWHEEL");
        dashTelemetry.addData("current RPM (ROTATIONS PER MINUTE): ", rawRPM);
        dashTelemetry.addData("current RPM Smooth (ROTATIONS PER MINUTE): ", currentRPMSmooth);
        dashTelemetry.addData("current power (PERCENTAGE): ", pidf);

        dashTelemetry.addData("current pos (TICKS): ", encoder.getPosition());
        dashTelemetry.addData("target RPM (ROTATIONS PER MINUTE): ", shootingRPM);

    }
}
