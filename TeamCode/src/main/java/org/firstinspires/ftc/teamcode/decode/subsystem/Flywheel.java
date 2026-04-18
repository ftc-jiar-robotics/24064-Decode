package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FLYWHEEL_MASTER_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FLYWHEEL_SLAVE_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFlywheelManual;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Turret.calculateTurretPosition;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.IIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.LowPassGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.MovingAverageGains;
import org.firstinspires.ftc.teamcode.decode.control.solverscontrol.SolversPIDF;
import org.firstinspires.ftc.teamcode.decode.util.CachedMotor;

@Configurable
@Config
public class Flywheel extends Subsystem<Flywheel.FlyWheelStates> {
    private final CachedMotor[] motorGroup;
    private final Motor.Encoder shooterEncoder;
    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS_CLOSE = new PIDFCoefficients(0.0022, 0.000, 0.00005, 0.000077);
    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS_FAR = new PIDFCoefficients(0.0029, 0.0003, 0.00002, 0.000077);
    private final SolversPIDF velocityController = new SolversPIDF(FLYWHEEL_PIDF_COEFFICIENTS_CLOSE);
    public static final double GEAR_RATIO = 20.0/20;

    public enum FlyWheelStates {
        IDLE, ARMING, RUNNING
    }
    public static double
            MAX_RPM = 4000,
            RPM_PER_SEC_IN = 8.17067, // TODO EMPIRICALLY TUNE
            RPM_TOLERANCE = 70,
            LOW_PASS_FILTER_RPM_TOLERANCE = 250,
            SMOOTH_RPM_GAIN = 0,
            DERIV_TOLERANCE = 600,
            IDLE_RPM = 1200,
            BB_TOLERANCE = 150000000,
            BB_ENABLE_DISTANCE = 110,
            TARGET_RPM_STEP = 30.0,
            TARGET_RPM_MID_BAND = 9.0,
            SWITCH_PID_DIST = 100, // inches to switch to far PID
            kS = 0.25,
            ROUNDING_POINT = 100;

    private FlyWheelStates targetState = FlyWheelStates.IDLE;

    public static LowPassGains motorPowerGains = new LowPassGains(
            0,
            6);

    private final IIRLowPassFilter motorPowerFilter = new IIRLowPassFilter(motorPowerGains);

    public boolean
            movingToFarZone,
            ballIsPresent;

    private double
            currentRPM = 0.0,
            currentRPMSmooth = 0.0,
            manualPower = 0.0,
            shootingRPM = 4000,
            currentPower = 0;

    private static double quantizeWithMidpointBand(double rpmRaw, double step, double band) {
        double low = Math.floor(rpmRaw / step) * step;
        double high = low + step;
        double mid = (low + high) / 2.0;

        // If we're near the midpoint (like 2375), use midpoint so it doesn't flip bins
        if (Math.abs(rpmRaw - mid) <= band) return mid;

        // Otherwise normal rounding to nearest step
        return Math.round(rpmRaw / step) * step;
    }

    public Flywheel(HardwareMap hw) {
        CachedMotor shooterMaster = new CachedMotor(hw, NAME_FLYWHEEL_MASTER_MOTOR, Motor.GoBILDA.BARE,ROUNDING_POINT);
        CachedMotor shooterSlave = new CachedMotor(hw, NAME_FLYWHEEL_SLAVE_MOTOR, Motor.GoBILDA.BARE,ROUNDING_POINT);
        MotorEx dummy = new MotorEx(hw, NAME_FLYWHEEL_MASTER_MOTOR, Motor.GoBILDA.BARE);

        shooterSlave.setInverted(false);
        shooterMaster.setInverted(true);

        shooterEncoder = dummy.encoder;
        shooterEncoder.setDirection(Motor.Direction.FORWARD);

        motorGroup = new CachedMotor[]{shooterMaster, shooterSlave};
    }

    @Override
    public void set(FlyWheelStates f) {
        targetState = f;
    }

    @Override
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
        currentRPM = (shooterEncoder.getCorrectedVelocity() * 60.0 / 28.0);
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
                break;
            case RUNNING:
                quantizeWithMidpointBand(robot.shooter.getCompensatedValues()[0], TARGET_RPM_STEP, TARGET_RPM_MID_BAND);
                break;
        }

        double feedforwardValue = 0;

        PIDFCoefficients coefficients = robot.shooter.turret.getDistance() <= SWITCH_PID_DIST ?
                FLYWHEEL_PIDF_COEFFICIENTS_CLOSE :
                FLYWHEEL_PIDF_COEFFICIENTS_FAR;

        double originalKf = coefficients.f;
        coefficients.f  = originalKf*(Common.MAX_VOLTAGE / robot.getVoltage());

        velocityController.setCoefficients(coefficients);

        currentPower = feedforwardValue + kS*(Common.MAX_VOLTAGE / robot.getVoltage());
        currentPower += velocityController.calculate(currentRPMSmooth);

        if (Math.abs(currentRPMSmooth - shootingRPM) < LOW_PASS_FILTER_RPM_TOLERANCE) {
            currentPower = motorPowerFilter.calculate(currentPower);
        }
        else {
            motorPowerFilter.reset();
        }

        coefficients.f = originalKf;

        currentPower = Range.clip(currentPower, feedforwardValue, 1.0);

        for (MotorEx m : motorGroup) {
            if (shootingRPM-currentRPMSmooth>BB_TOLERANCE && robot.shooter.turret.getDistance()>BB_ENABLE_DISTANCE) m.set(1);
            else m.set(Math.abs(manualPower) > 0 ? manualPower : currentPower);
        }

        if (isPIDInTolerance() && robot.shooter.getQueuedShots() <= 0) velocityController.reset();
    }

    public void incrementFlywheelRPM(double RPM, boolean isIncrementing) {
        if (isIncrementing) shootingRPM += RPM;
        else shootingRPM -= RPM;

        velocityController.setSetPoint(shootingRPM);
    }

    public static double inchesPerSecondToRPM(double x) {
        return x * RPM_PER_SEC_IN;
    }

    public static double RPMToInchesPerSecond(double x) {
        return x / RPM_PER_SEC_IN;
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
