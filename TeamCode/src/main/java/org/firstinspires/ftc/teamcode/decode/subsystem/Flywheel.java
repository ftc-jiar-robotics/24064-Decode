package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FLYWHEEL_MASTER_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FLYWHEEL_SLAVE_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFlywheelManual;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFuturePoseOn;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Turret.calculateTurretPosition;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.Filter;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.IIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.LowPassGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.MovingAverageGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.Differentiator;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;
import org.firstinspires.ftc.teamcode.decode.util.CachedMotor;

@Configurable
@Config
public class Flywheel extends Subsystem<Flywheel.FlyWheelStates> {
    private final DcMotorEx[] motorGroup;

    private final Motor.Encoder shooterEncoder;

    private final PIDController velocityController = new PIDController();

    public static PIDGains shootingVelocityGains = new PIDGains(
            600,
            50,
            0,
            Double.POSITIVE_INFINITY
    );

    public static PIDGains shootingWhileMovingVelocityGains = new PIDGains(
            600,
            50,
            0,
            Double.POSITIVE_INFINITY
    );

    private final FIRLowPassFilter rpmFilter = new FIRLowPassFilter();
   // public static MovingAverageGains rpmDerivAverageFilterGains = new MovingAverageGains(3);
    public static MovingAverageGains targetRPMAverageFilterGains = new MovingAverageGains(
            5
    );

    //private final Filter targetRPMAverageFilter = new MovingAverageFilter(targetRPMAverageFilterGains);

    public static LowPassGains rpmFilterGains = new LowPassGains(
            0.5,
            10
    );



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
            SMOOTH_RPM_GAIN = .9,
            DERIV_TOLERANCE = 200,
            IDLE_RPM = 1200,
            FAR_ARMING_RPM = 2950,
            CLOSE_ARMING_RPM = 2100,
            MAX_RPM = 4000,
            VOLTAGE_SCALER = 0,
            TARGET_RPM_STEP = 30.0,
            TARGET_RPM_MID_BAND = 9.0;

    private FlyWheelStates targetState = FlyWheelStates.IDLE;

    public static LowPassGains motorPowerGains = new LowPassGains(
            0.4,
            6);

    private final IIRLowPassFilter motorPowerFilter = new IIRLowPassFilter(motorPowerGains);

    private boolean isDirectionForward = false;

    private double
            currentRPM = 0.0,
            notInToleranceCounter = 0,
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
//        MotorEx shooterMaster = new MotorEx(hw, NAME_FLYWHEEL_MASTER_MOTOR, Motor.GoBILDA.BARE);
//        MotorEx shooterSlave = new MotorEx(hw, NAME_FLYWHEEL_SLAVE_MOTOR, Motor.GoBILDA.BARE);
        MotorEx dummy = new MotorEx(hw, NAME_FLYWHEEL_MASTER_MOTOR, Motor.GoBILDA.BARE);

        DcMotorEx shooterMaster = hw.get(DcMotorEx.class,NAME_FLYWHEEL_MASTER_MOTOR);
        DcMotorEx shooterSlave = hw.get(DcMotorEx.class, NAME_FLYWHEEL_SLAVE_MOTOR);


//        shooterSlave.setInverted(false);
        shooterMaster.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMaster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterSlave.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMaster.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(shootingVelocityGains.kP,shootingVelocityGains.kI,shootingVelocityGains.kD,0));
        shooterSlave.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(shootingVelocityGains.kP,shootingVelocityGains.kI,shootingVelocityGains.kD,0));


        shooterEncoder = dummy.encoder;
        shooterEncoder.setDirection(Motor.Direction.FORWARD);

        motorGroup = new DcMotorEx[]{shooterMaster, shooterSlave};

       // velocityController.setDerivativeMode(PIDController.DerivativeMode.MEASUREMENT);
        velocityController.setGains(shootingVelocityGains);
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

    public void setManualPower(double power) {
        manualPower = power;
    }

    public boolean isPIDInTolerance() {
        return (velocityController.isInTolerance(new State(currentRPMSmooth, 0, 0, 0), robot.isRobotMoving() ? RPM_TOLERANCE_WHILE_MOVING : RPM_TOLERANCE, DERIV_TOLERANCE));
    }

    @Override
    public void run() {
        currentRPM = (shooterEncoder.getCorrectedVelocity() * 60.0 / 28.0);
        currentRPMSmooth = (SMOOTH_RPM_GAIN * currentRPMSmooth) + (1 - SMOOTH_RPM_GAIN) * currentRPM;
        if (currentRPM > 10000) currentRPM = 0;
        if (currentRPMSmooth > 10000) currentRPMSmooth = 0;

        motorPowerFilter.setGains(motorPowerGains);

        if (robot.isRobotMoving(MIN_MOVEMENT_SPEED)) velocityController.setGains(shootingWhileMovingVelocityGains);
        else velocityController.setGains(shootingVelocityGains);

        switch (targetState) {
            case IDLE:
                boolean isRobotCloseToFar = robot.drivetrain.getPose().getY() < 40;
                boolean isMagnitudeInPositiveTolerance = robot.drivetrain.getVelocity().getYComponent() > 0.3;
                boolean isMagnitudeInNegativeTolerance = robot.drivetrain.getVelocity().getYComponent() < -0.3;

                if (isMagnitudeInPositiveTolerance) isDirectionForward = true;
                else if (isMagnitudeInNegativeTolerance) isDirectionForward = false;

                if (!isFlywheelManual) shootingRPM = robot.shooter.isBallPresent() ? (isRobotCloseToFar && !isDirectionForward ? FAR_ARMING_RPM : CLOSE_ARMING_RPM) : IDLE_RPM;
                velocityController.setTarget(new State(shootingRPM, 0, 0, 0));

                break;
            case ARMING:
                shootingRPM = quantizeWithMidpointBand(inchesPerSecondToRPM(robot.shooter.kinematicsSolver.v_launch), TARGET_RPM_STEP, TARGET_RPM_MID_BAND);
                velocityController.setTarget(new State(shootingRPM, 0, 0, 0));

                if (isPIDInTolerance()) targetState = FlyWheelStates.RUNNING;
                break;
            case RUNNING:
                shootingRPM = quantizeWithMidpointBand(inchesPerSecondToRPM(robot.shooter.kinematicsSolver.v_launch), TARGET_RPM_STEP, TARGET_RPM_MID_BAND);
                velocityController.setTarget(new State(shootingRPM, 0, 0, 0));

                break;
        }

//        double feedforwardValue = (shootingRPM/MAX_RPM) * (Math.sqrt(Common.MAX_VOLTAGE) / Math.sqrt(robot.batteryVoltageSensor.getVoltage())) * VOLTAGE_SCALER;
//
//        currentPower = feedforwardValue;
//        currentPower += velocityController.calculate(new State(currentRPMSmooth, 0, 0, 0));
//
//        if (Math.abs(currentRPMSmooth - shootingRPM) < LOW_PASS_FILTER_RPM_TOLERANCE) {
//            currentPower = motorPowerFilter.calculate(currentPower);
//            notInToleranceCounter = 0;
//        }
//        else {
//            motorPowerFilter.reset();
//            notInToleranceCounter++;
//        }
//
//        currentPower = Range.clip(currentPower, feedforwardValue/2, 1.0);

        for (DcMotorEx m : motorGroup) m.setVelocity(shootingRPM*28.0/60.0);

        if (isPIDInTolerance() && robot.shooter.getQueuedShots() <= 0) velocityController.reset();
    }

    public void incrementFlywheelRPM(double RPM, boolean isIncrementing) {
        if (isIncrementing) shootingRPM += RPM;
        else shootingRPM -= RPM;

        velocityController.setTarget(new State(shootingRPM, 0, 0, 0));
    }

    public static double inchesPerSecondToRPM(double x) {
        return (0.0571374 * x * x) + (-6.80299 * x) + 985.1611;
    }

    public static double RPMToInchesPerSecond(double x) {
        return (-0.0000175444 * x * x) + (0.13418 * x) + 8.4288;
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
