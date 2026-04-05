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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.SISOKalmanFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.KalmanGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;

@Configurable
@Config
public class Flywheel extends Subsystem<Flywheel.FlyWheelStates> {

    private final CachedMotorEx[] motors;
    private final Motor.Encoder shooterEncoder;

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
            VOLTAGE_SCALAR = .9,
            TARGET_RPM_STEP = 30.0,
            TARGET_RPM_MID_BAND = 9.0;

    private FlyWheelStates targetState = FlyWheelStates.IDLE;

    public static KalmanGains
            rpmFilterGains = new KalmanGains(1, 2),
            derivFilterGains = new KalmanGains(3, 0);
    private final SISOKalmanFilter
            rpmFilter = new SISOKalmanFilter(rpmFilterGains),
            derivFilter = new SISOKalmanFilter(derivFilterGains);

    public static PIDGains FLYWHEEL_PIDF_COEFFICIENTS = new PIDGains(0.0067, 0, 0, 1);
    private final PIDController velocityController = new PIDController(derivFilter);

    private boolean isDirectionForward = false;

    private double rawRPM, currentRPMSmooth, manualPower, currentPower, shootingRPM = 4000;

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

        shooterEncoder = new MotorEx(hw, NAME_FLYWHEEL_MASTER_MOTOR, Motor.GoBILDA.BARE).encoder;
        shooterEncoder.setDirection(Motor.Direction.FORWARD);

        motors = new CachedMotorEx[]{
                new CachedMotorEx(hw, NAME_FLYWHEEL_MASTER_MOTOR, Motor.GoBILDA.BARE).reversed(),
                new CachedMotorEx(hw, NAME_FLYWHEEL_SLAVE_MOTOR, Motor.GoBILDA.BARE)
        };

        rpmFilter.setGains(rpmFilterGains);
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

//    public double getError() {
//        return velocityController.getError();
//    }


    public void setManualPower(double power) {
        manualPower = power;
    }

    public boolean isPIDInTolerance() {
//        return (velocityController.isInTolerance(new State(currentRPMSmooth, 0, 0, 0), robot.isRobotMoving() ? RPM_TOLERANCE_WHILE_MOVING : RPM_TOLERANCE, DERIV_TOLERANCE));

        double tolerance = robot.isRobotMoving() ? RPM_TOLERANCE_WHILE_MOVING : RPM_TOLERANCE;
        double derivTolerance = DERIV_TOLERANCE;

        abs(velocityController.getError()) <= tolerance && velocityController.

        return velocityController.atSetPoint();
    }


    @Override
    public void run() {
        rawRPM = (shooterEncoder.getCorrectedVelocity() * 60.0 / 28.0);
        currentRPMSmooth = rpmFilter.calculate(rawRPM);

        if (rawRPM > 10000) rawRPM = 0;
        if (currentRPMSmooth > 10000) currentRPMSmooth = 0;

        motorPowerFilter.setGains(motorPowerGains);

        velocityController.setCoefficients(FLYWHEEL_PIDF_COEFFICIENTS);

        switch (targetState) {
            case IDLE:
                boolean isRobotCloseToFar = robot.drivetrain.getPose().getY() < 40;
                boolean isMagnitudeInPositiveTolerance = robot.drivetrain.getVelocity().getYComponent() > 0.3;
                boolean isMagnitudeInNegativeTolerance = robot.drivetrain.getVelocity().getYComponent() < -0.3;

                if (isMagnitudeInPositiveTolerance) isDirectionForward = true;
                else if (isMagnitudeInNegativeTolerance) isDirectionForward = false;

                if (!isFlywheelManual) shootingRPM =
                        !robot.shooter.isBallPresent() ?            IDLE_RPM :
                        isRobotCloseToFar && !isDirectionForward ?  FAR_ARMING_RPM :
                                                                    CLOSE_ARMING_RPM;
                velocityController.setSetPoint(shootingRPM);

                break;
            case ARMING:
                shootingRPM = quantizeWithMidpointBand(inchesPerSecondToRPM(robot.shooter.getCompensatedValues()[0]), TARGET_RPM_STEP, TARGET_RPM_MID_BAND);
                velocityController.setSetPoint(shootingRPM);

                if (isPIDInTolerance()) targetState = FlyWheelStates.RUNNING;
                break;
            case RUNNING:
                shootingRPM = quantizeWithMidpointBand(inchesPerSecondToRPM(robot.shooter.getCompensatedValues()[0]), TARGET_RPM_STEP, TARGET_RPM_MID_BAND);
                velocityController.setSetPoint(shootingRPM);

                break;
        }


        double feedforwardValue = 0;//(shootingRPM/MAX_RPM) * (Math.sqrt(Common.MAX_VOLTAGE) / Math.sqrt(robot.batteryVoltageSensor.getVoltage())) * VOLTAGE_SCALER;

        currentPower = feedforwardValue;
        currentPower += velocityController.calculate(currentRPMSmooth);

        if (abs(currentRPMSmooth - shootingRPM) < LOW_PASS_FILTER_RPM_TOLERANCE) {
            currentPower = motorPowerFilter.calculate(currentPower);
            notInToleranceCounter = 0;
        }
        else {
            motorPowerFilter.reset();
            notInToleranceCounter++;
        }

        currentPower = Range.clip(currentPower, feedforwardValue, 1.0);

//        if (robot.shooter.get() == Shooter.ShooterStates.RUNNING) currentPower = motorGroup[0].get();

        for (MotorEx m : motors) {
            if (shootingRPM-currentRPMSmooth>BB_TOLERANCE) m.set(1);
            else m.set(abs(manualPower) > 0 ? manualPower : currentPower);
        }


//        for (DcMotorEx m : motorGroup) {
//
//            m.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(shootingVelocityGains.kP,shootingVelocityGains.kI,shootingVelocityGains.kD,feedforwardValue));
//            m.setVelocity(shootingRPM*28.0/60.0);
//        }


        if (isPIDInTolerance() && robot.shooter.getQueuedShots() <= 0) velocityController.reset();
    }

    public void incrementFlywheelRPM(double RPM, boolean isIncrementing) {
        if (isIncrementing) shootingRPM += RPM;
        else shootingRPM -= RPM;

        velocityController.setSetPoint(shootingRPM);
    }

    public static double inchesPerSecondToRPM(double x) {
        return (0.0571374 * x * x) + (-6.80299 * x) + 985.1611;
    }

    public static double RPMToInchesPerSecond(double x) {
        return (-0.0000175444 * x * x) + (0.13418 * x) + 8.4288;
    }

    private void chooseShootingRPM(double distance) {
//        shootingRPM = lutRPM[0];
//        for (int i = 0; i < lutDistances.length; i++) {
//            if (Common.robot.shooter.turret.getDistance() >= lutDistances[i]) shootingRPM = lutRPM[i];
//        }
        if (!isFlywheelManual) {
            double rpmRaw = 957.2952559300876*(1) + 14.312109862671662*(distance);
            shootingRPM = quantizeWithMidpointBand(rpmRaw, TARGET_RPM_STEP, TARGET_RPM_MID_BAND);
            velocityController.setSetPoint(shootingRPM);
        }
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
//        dashTelemetry.addData("current power 1 (PERCENTAGE): ", motorGroup[0].getPower());
//        dashTelemetry.addData("current power 2 (PERCENTAGE): ", motorGroup[1].getPower());
        dashTelemetry.addData("current power (PERCENTAGE): ", currentPower);

        dashTelemetry.addData("current pos (TICKS): ", shooterEncoder.getPosition());
        dashTelemetry.addData("target RPM (ROTATIONS PER MINUTE): ", shootingRPM);

    }
}
