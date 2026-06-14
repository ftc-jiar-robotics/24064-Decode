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
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.IIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.LowPassGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.MovingAverageGains;
import org.firstinspires.ftc.teamcode.decode.control.solverscontrol.SolversPIDF;
import org.firstinspires.ftc.teamcode.decode.util.CachedMotor;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;
import org.firstinspires.ftc.teamcode.decode.util.solverslib.InterpLUT;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.decode.util.solverslib.InterpLUT;

import java.util.Arrays;
import java.util.List;

@Configurable
@Config
public class Flywheel extends Subsystem<Flywheel.FlyWheelStates> {
    private final CachedMotor[] motorGroup;
    private final Motor.Encoder shooterEncoder;
    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS_CLOSE = new PIDFCoefficients(0.0027, 0.000, 0.0000, 0.00005);
    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS_CLOSE_AUTON = new PIDFCoefficients(0.0027, 0.000, 0.0000, 0.00005);

    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS_FAR = new PIDFCoefficients(.008, 0.000, 0.0000, 0.000087);
    private final SolversPIDF velocityController = new SolversPIDF(FLYWHEEL_PIDF_COEFFICIENTS_CLOSE);

    private static final List<Double> launcherDistance = Arrays.asList(0.0,  /*59.055,  78.740, 98.425, 118.110, 137.795, 157.480, 177.165,*/ 196.850); // distance from ball leaving robot to when it touches goal for first time (inches)
    private static final List<Double> shootingTime     = Arrays.asList(0.3, /*0.58375, 0.5,    0.52,    0.70,    0.77,    0.80d,   0.83d,*/   0.86); // time it takes for ball to leave robot to start of goal (seconds)

    private final InterpLUT launchDelayLUT = new InterpLUT(launcherDistance,shootingTime);
    public static final double GEAR_RATIO = 20.0/20;

    public static List<Double> distanceValuesLUT = Arrays.asList(0.0, 160.0);  // distance in inches
    public static List<Double> rpmValuesLUT = Arrays.asList(8.5, 10.8); // in IPS; multipler

    private final InterpLUT rpmInchPerSecLUT = new InterpLUT(distanceValuesLUT, rpmValuesLUT);

    public enum FlyWheelStates {
        IDLE, ARMING, RUNNING
    }
    public static double
            LAUNCH_DELAY = 0.3,
            OUT_OF_TOLERANCE_LOOPS = 3,
            RPM_TOLERANCE = 50,
            MAX_RPM = 4000,
            RPM_PER_SEC_IN = 12.00567, // TODO EMPIRICALLY TUNE
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
            CLOSE_ADJUSTMENT_RPM = 70, // added onto rpm curve
            FAR_ADJUSTMENT_RPM = 700,
            TURRET_ANGLE_MULTIPLIER = .1,
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

        shooterSlave.setInverted(true);
        shooterMaster.setInverted(false);

        shooterEncoder = dummy.encoder;
        shooterEncoder.setDirection(Motor.Direction.FORWARD);

        motorGroup = new CachedMotor[]{shooterMaster, shooterSlave};

        launchDelayLUT.createLUT();
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

    public double getTargetRPM() {
        return shootingRPM;
    }

    public void setManualPower(double power) {
        manualPower = power;
    }

    public boolean isPIDInTolerance() {
//        return (velocityController.isInTolerance(new State(currentRPMSmooth, 0, 0, 0), robot.isRobotMoving() ? RPM_TOLERANCE_WHILE_MOVING : RPM_TOLERANCE, DERIV_TOLERANCE));

        velocityController.setTolerance(/*robot.isRobotMoving() ? RPM_TOLERANCE_WHILE_MOVING : */RPM_TOLERANCE, DERIV_TOLERANCE);

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
//                boolean isRobotCloseToFar = robot.drivetrain.getPose().getY() < 40;
//                boolean isMagnitudeInPositiveTolerance = robot.drivetrain.getVelocity().getYComponent() > 0.3;
//                boolean isMagnitudeInNegativeTolerance = robot.drivetrain.getVelocity().getYComponent() < -0.3;
//
//                if (isMagnitudeInPositiveTolerance) isDirectionForward = true;
//                else if (isMagnitudeInNegativeTolerance) isDirectionForward = false;

                if (!isFlywheelManual) {
                    if (robot.shooter.isBallPresent()) {
                        if (!robot.usingSotm()) chooseShootingRPM(robot.shooter.turret.getDistance(calculateTurretPosition(LaunchZone.getInterceptOrClosestPoint(), Math.toDegrees(robot.drivetrain.getHeading()), -Common.TURRET_OFFSET_Y)));
                        else shootingRPM = quantizeWithMidpointBand(inchesPerSecondToRPM(robot.shooter.getCompensatedValues()[0]), TARGET_RPM_STEP, TARGET_RPM_MID_BAND);
                    }
                    else shootingRPM =  IDLE_RPM;
                }
                velocityController.setSetPoint(shootingRPM);

                break;
            case ARMING:
                if (!robot.usingSotm()) chooseShootingRPM(robot.shooter.turret.getDistance(calculateTurretPosition(robot.drivetrain.getPose(), Math.toDegrees(robot.drivetrain.getHeading()), -Common.TURRET_OFFSET_Y)));
                else shootingRPM = quantizeWithMidpointBand(inchesPerSecondToRPM(robot.shooter.getCompensatedValues()[0]), TARGET_RPM_STEP, TARGET_RPM_MID_BAND);
                if (isPIDInTolerance()) targetState = FlyWheelStates.RUNNING;
                break;
            case RUNNING:
                if (!robot.usingSotm()) chooseShootingRPM(robot.shooter.turret.getDistance(calculateTurretPosition(robot.drivetrain.getPose(), Math.toDegrees(robot.drivetrain.getHeading()), -Common.TURRET_OFFSET_Y)));
                else shootingRPM = quantizeWithMidpointBand(inchesPerSecondToRPM(robot.shooter.getCompensatedValues()[0]), TARGET_RPM_STEP, TARGET_RPM_MID_BAND);
                break;
        }

        velocityController.setSetPoint(shootingRPM);

        double feedforwardValue = 0;//(shootingRPM/MAX_RPM) * (Math.sqrt(Common.MAX_VOLTAGE) / Math.sqrt(robot.batteryVoltageSensor.getVoltage())) * VOLTAGE_SCALER;

        PIDFCoefficients coefficients = robot.shooter.turret.getDistance() <= SWITCH_PID_DIST ?
                (robot.isAuto ? FLYWHEEL_PIDF_COEFFICIENTS_CLOSE_AUTON : FLYWHEEL_PIDF_COEFFICIENTS_CLOSE) :
                FLYWHEEL_PIDF_COEFFICIENTS_FAR;

//        double originalKd = coefficients.d;
        double originalKf = coefficients.f;
//        double originalKp = coefficients.p;
//        coefficients.d  = originalKd*(Common.MAX_VOLTAGE / robot.getVoltage());
        coefficients.f  = originalKf*(Common.MAX_VOLTAGE / robot.getVoltage());
//        coefficients.p  = originalKp*(Common.MAX_VOLTAGE / robot.getVoltage());

        velocityController.setCoefficients(coefficients);

        currentPower = feedforwardValue + kS*(Common.MAX_VOLTAGE / robot.getVoltage());
        currentPower += velocityController.calculate(currentRPMSmooth);

        if (Math.abs(currentRPMSmooth - shootingRPM) < LOW_PASS_FILTER_RPM_TOLERANCE) {
            currentPower = motorPowerFilter.calculate(currentPower);
        }
        else {
            motorPowerFilter.reset();
        }

//        coefficients.d = originalKd;
        coefficients.f = originalKf;
//        coefficients.p = originalKp;


        currentPower = Range.clip(currentPower, feedforwardValue, 1.0);

//        if (robot.shooter.get() == Shooter.ShooterStates.RUNNING) currentPower = motorGroup[0].get();

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


    public double inchesPerSecondToRPM(double x) {
        return x * RPM_PER_SEC_IN;
    }

    public double RPMToInchesPerSecond(double x) {
        return x / RPM_PER_SEC_IN;
    }

    private void chooseShootingRPM(double distance) {
//        shootingRPM = lutRPM[0];
//        for (int i = 0; i < lutDistances.length; i++) {
//            if (Common.robot.shooter.turret.getDistance() >= lutDistances[i]) shootingRPM = lutRPM[i];
//        }
        if (!isFlywheelManual) {
            double rpmRaw = GEAR_RATIO*(1674.6095342210476*(1) + -4.328034369655546*(distance) + 0.08095436010080519*(distance*distance));
            if (robot.isFar) rpmRaw+=FAR_ADJUSTMENT_RPM;
            else rpmRaw+=robot.isAuto ? 50 : CLOSE_ADJUSTMENT_RPM;

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
//        dashTelemetry.addData("current power 1 (PERCENTAGE): ", motorGroup[0].getPower());
//        dashTelemetry.addData("current power 2 (PERCENTAGE): ", motorGroup[1].getPower());
        dashTelemetry.addData("current power (PERCENTAGE): ", currentPower);

        dashTelemetry.addData("current pos (TICKS): ", shooterEncoder.getPosition());
        dashTelemetry.addData("target RPM (ROTATIONS PER MINUTE): ", shootingRPM);

    }
}
