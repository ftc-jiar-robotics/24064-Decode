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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.IIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.LowPassGains;
import org.firstinspires.ftc.teamcode.decode.control.solverscontrol.SolversPIDF;

@Configurable
@Config
public class Flywheel extends Subsystem<Flywheel.FlyWheelStates> {
    private final MotorEx[] motorGroup;
    private final Motor.Encoder shooterEncoder;
    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS_CLOSE = new PIDFCoefficients(0.0022, 0, 0.00005, 0.000077);
    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS_FAR = new PIDFCoefficients(0.0022, 0, 0.00005, 0.000077);
    private final SolversPIDF velocityController = new SolversPIDF(FLYWHEEL_PIDF_COEFFICIENTS_CLOSE);
    public static final double GEAR_RATIO = 20.0/20;

    public enum FlyWheelStates {
        IDLE, ARMING, RUNNING
    }

    public static LowPassGains motorPowerGains = new LowPassGains(
            0,
            6
    );

    private final IIRLowPassFilter motorPowerFilter = new IIRLowPassFilter(motorPowerGains);

    public static double
            RPM_PER_SEC_IN = 8.17067, // TODO EMPIRICALLY TUNE
            LAUNCH_DELAY = 0.3,
            RPM_TOLERANCE = 100,
            SMOOTH_RPM_GAIN = 0,
            LOW_PASS_FILTER_RPM_TOLERANCE = 250,
            DERIV_TOLERANCE = 600,
            IDLE_RPM = 1200,
            BB_TOLERANCE = 1000000,
            BB_ENABLE_DISTANCE = 110,
            TARGET_RPM_STEP = 30.0,
            TARGET_RPM_MID_BAND = 9.0,
            SWITCH_PID_DIST = 100, // inches to switch to far PID
            kS = 0.25,
            CLOSE_ADJUSTMENT_RPM = 25, // added onto rpm curve
            FAR_ADJUSTMENT_RPM = 40;

    private double
        currentRPM = 0.0,
        currentRPMSmooth = 0.0,
        manualPower = 0.0,
        shootingRPM = 4000,
        currentPower = 0;

    private FlyWheelStates targetState = FlyWheelStates.IDLE;
    boolean ballIsPresent, movingToFarZone;

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
        MotorEx shooterMaster = new MotorEx(hw, NAME_FLYWHEEL_MASTER_MOTOR, Motor.GoBILDA.BARE);
        MotorEx shooterSlave = new MotorEx(hw, NAME_FLYWHEEL_SLAVE_MOTOR, Motor.GoBILDA.BARE);
        MotorEx dummy = new MotorEx(hw, NAME_FLYWHEEL_MASTER_MOTOR, Motor.GoBILDA.BARE);

        shooterSlave.setInverted(false);
        shooterMaster.setInverted(true);

        shooterEncoder = dummy.encoder;
        shooterEncoder.setDirection(Motor.Direction.FORWARD);

        motorGroup = new MotorEx[]{shooterMaster, shooterSlave};
    }

    public void set(FlyWheelStates f) {
        targetState = f;
    }
    public FlyWheelStates get() {
        return targetState;
    }

    public double getCurrentRPMSmooth() {
        return currentRPMSmooth;
    }

    public void setManualPower(double power) {
        manualPower = power;
    }

    public boolean isPIDInTolerance() {
        velocityController.setTolerance(RPM_TOLERANCE, DERIV_TOLERANCE);

        return velocityController.atSetPoint();
    }

    @Override
    public void run() {
        currentRPM = (shooterEncoder.getCorrectedVelocity() * 60.0 / 20.0);
        currentRPMSmooth = (SMOOTH_RPM_GAIN * currentRPMSmooth) + (1 - SMOOTH_RPM_GAIN) * currentRPM;
        if (currentRPM > 10000) currentRPM = 0;
        if (currentRPMSmooth > 10000) currentRPMSmooth = 0;

        motorPowerFilter.setGains(motorPowerGains);

        switch (targetState) {
            case IDLE:
                if (!isFlywheelManual) {
                    if (robot.shooter.isBallPresent()) quantizeWithMidpointBand(robot.shooter.getCompensatedValues()[0], TARGET_RPM_STEP, TARGET_RPM_MID_BAND);
                    else shootingRPM =  IDLE_RPM;
                }

                velocityController.setSetPoint(shootingRPM);
                break;
            case ARMING:
                quantizeWithMidpointBand(robot.shooter.getCompensatedValues()[0], TARGET_RPM_STEP, TARGET_RPM_MID_BAND);
                if (isPIDInTolerance()) targetState = FlyWheelStates.RUNNING;
            case RUNNING:
                quantizeWithMidpointBand(robot.shooter.getCompensatedValues()[0], TARGET_RPM_STEP, TARGET_RPM_MID_BAND);
                break;
        }

        double feedforwardValue = 0;

        PIDFCoefficients coefficients = robot.shooter.turret.getDistance() <= SWITCH_PID_DIST ?
                FLYWHEEL_PIDF_COEFFICIENTS_CLOSE :
                FLYWHEEL_PIDF_COEFFICIENTS_FAR;

        double originalKf = coefficients.f;
        coefficients.f  = originalKf*(Common.MAX_VOLTAGE / robot.batteryVoltageSensor.getVoltage());

        velocityController.setCoefficients(coefficients);

        currentPower = feedforwardValue + kS*(Common.MAX_VOLTAGE / robot.batteryVoltageSensor.getVoltage());
        currentPower += velocityController.calculate(currentRPMSmooth);

        if (Math.abs(currentRPMSmooth - shootingRPM) < LOW_PASS_FILTER_RPM_TOLERANCE) currentPower = motorPowerFilter.calculate(currentPower);
        else motorPowerFilter.reset();

        coefficients.f = originalKf;

        currentPower = Range.clip(currentPower, feedforwardValue, 1.0);

        for (MotorEx m : motorGroup) {
            if (shootingRPM-currentRPMSmooth>BB_TOLERANCE && robot.shooter.turret.getDistance()>BB_ENABLE_DISTANCE) m.set(1);
            else m.set(Math.abs(manualPower) > 0 ? manualPower : currentPower);
        }

        if (isPIDInTolerance() && robot.shooter.getQueuedShots() <= 0) velocityController.reset();
    }

    public void incrementFlywheelRPM(double RPM) {
        shootingRPM += RPM;
        velocityController.setSetPoint(shootingRPM);
    }

    public static double inchesPerSecondToRPM(double x) {
        return x * RPM_PER_SEC_IN;
    }

    public static double RPMToInchesPerSecond(double x) {
        return x / RPM_PER_SEC_IN;
    }

    private void chooseShootingRPM(double distance) {
        if (!isFlywheelManual) {
            double rpmRaw = GEAR_RATIO*(1591.965085639697*(1) + -2.1583397782159177*(distance) + 0.06810134596813792*(distance*distance));
            if (robot.isFar) rpmRaw+=FAR_ADJUSTMENT_RPM;
            else rpmRaw+=CLOSE_ADJUSTMENT_RPM;
            shootingRPM = quantizeWithMidpointBand(rpmRaw, TARGET_RPM_STEP, TARGET_RPM_MID_BAND);

            velocityController.setSetPoint(shootingRPM);
        }
    }


    public void printTelemetry() {
        telemetry.addLine("FLYWHEEL");
        telemetry.addData("current state (ENUM): ", get());
        telemetry.addData("target state (ENUM): ", targetState);
        telemetry.addData("current RPM (ROTATIONS PER MINUTE): ", currentRPM);
        telemetry.addData("is PID in tolerance (BOOLEAN): ", isPIDInTolerance());

        dashTelemetry.addLine("FLYWHEEL");
        dashTelemetry.addData("current RPM (ROTATIONS PER MINUTE): ", currentRPM);
        dashTelemetry.addData("current RPM Smooth (ROTATIONS PER MINUTE): ", currentRPMSmooth);
        dashTelemetry.addData("current power (PERCENTAGE): ", currentPower);

        dashTelemetry.addData("current pos (TICKS): ", shooterEncoder.getPosition());
        dashTelemetry.addData("target RPM (ROTATIONS PER MINUTE): ", shootingRPM);

    }
}
