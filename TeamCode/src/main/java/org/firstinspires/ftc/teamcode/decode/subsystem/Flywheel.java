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
import org.firstinspires.ftc.teamcode.decode.control.solverscontrol.SolversPIDF;
import org.firstinspires.ftc.teamcode.decode.util.CachedMotor;

@Configurable
@Config
public class Flywheel extends Subsystem<Flywheel.FlyWheelStates> {
    private final MotorEx[] motorGroup;

    private final Motor.Encoder shooterEncoder;

//    public static PIDGains shootingVelocityGains = new PIDGains(
//            0.00425,
//            0.0,
//            0.000825,
//            Double.POSITIVE_INFINITY
//    );
//
//    public static PIDGains shootingWhileMovingVelocityGains = new PIDGains(
//            0.00425,
//            0.0,
//            0.000825,
//            Double.POSITIVE_INFINITY
//    );

    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS = new PIDFCoefficients(0.0067, 0, 0, 0.00024);

    private final SolversPIDF velocityController = new SolversPIDF(FLYWHEEL_PIDF_COEFFICIENTS);
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
            MIN_MOVEMENT_SPEED = 35,
            LAUNCH_DELAY = 0.3,
            OUT_OF_TOLERANCE_LOOPS = 3,
            RPM_TOLERANCE = 70,
            LOW_PASS_FILTER_RPM_TOLERANCE = 250,
            RPM_TOLERANCE_WHILE_MOVING = 30,
            SMOOTH_RPM_GAIN = .9,
            DERIV_TOLERANCE = 100,
            IDLE_RPM = 1200,
            FAR_ARMING_RPM = 2950,
            CLOSE_ARMING_RPM = 2100,
            BB_TOLERANCE = 50000,
            MAX_RPM = 4000,
            VOLTAGE_SCALER = .9,
            TARGET_RPM_STEP = 30.0,
            TARGET_RPM_MID_BAND = 9.0;

    private FlyWheelStates targetState = FlyWheelStates.IDLE;

    public static LowPassGains motorPowerGains = new LowPassGains(
            0,
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
        MotorEx shooterMaster = new MotorEx(hw, NAME_FLYWHEEL_MASTER_MOTOR, Motor.GoBILDA.BARE);
        MotorEx shooterSlave = new MotorEx(hw, NAME_FLYWHEEL_SLAVE_MOTOR, Motor.GoBILDA.BARE);
        MotorEx dummy = new MotorEx(hw, NAME_FLYWHEEL_MASTER_MOTOR, Motor.GoBILDA.BARE);

//        DcMotorEx shooterMaster = hw.get(DcMotorEx.class,NAME_FLYWHEEL_MASTER_MOTOR);
//        DcMotorEx shooterSlave = hw.get(DcMotorEx.class, NAME_FLYWHEEL_SLAVE_MOTOR);


        shooterSlave.setInverted(false);
        shooterMaster.setInverted(true);
//        shooterMaster.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        shooterMaster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterSlave.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        shooterMaster.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(shootingVelocityGains.kP,shootingVelocityGains.kI,shootingVelocityGains.kD,0));
//        shooterSlave.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(shootingVelocityGains.kP,shootingVelocityGains.kI,shootingVelocityGains.kD,0));


        shooterEncoder = dummy.encoder;
        shooterEncoder.setDirection(Motor.Direction.FORWARD);

        motorGroup = new MotorEx[]{shooterMaster, shooterSlave};

       // velocityController.setDerivativeMode(PIDController.DerivativeMode.MEASUREMENT);
//        velocityController.setGains(shootingVelocityGains);
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

        velocityController.setTolerance(robot.isRobotMoving() ? RPM_TOLERANCE_WHILE_MOVING : RPM_TOLERANCE, DERIV_TOLERANCE);

        return velocityController.atSetPoint();
    }


    @Override
    public void run() {
        currentRPM = (shooterEncoder.getCorrectedVelocity() * 60.0 / 28.0);
        currentRPMSmooth = (SMOOTH_RPM_GAIN * currentRPMSmooth) + (1 - SMOOTH_RPM_GAIN) * currentRPM;
        if (currentRPM > 10000) currentRPM = 0;
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

                if (!isFlywheelManual) shootingRPM = robot.shooter.isBallPresent() ? (isRobotCloseToFar && !isDirectionForward ? FAR_ARMING_RPM : CLOSE_ARMING_RPM) : IDLE_RPM;
                velocityController.setSetPoint(shootingRPM);

                break;
            case ARMING:
                chooseShootingRPM(robot.shooter.turret.getDistance(calculateTurretPosition(/*robot.shooter.getPredictedPose(LAUNCH_DELAY)*/ robot.drivetrain.getPose(), Math.toDegrees(robot.drivetrain.getHeading()), -Common.TURRET_OFFSET_Y)));
                if (isPIDInTolerance()) targetState = FlyWheelStates.RUNNING;
                break;
            case RUNNING:
                chooseShootingRPM(robot.shooter.turret.getDistance(calculateTurretPosition(/*robot.shooter.getPredictedPose(LAUNCH_DELAY)*/ robot.drivetrain.getPose(), Math.toDegrees(robot.drivetrain.getHeading()), -Common.TURRET_OFFSET_Y)));
                break;
        }


        double feedforwardValue = 0;//(shootingRPM/MAX_RPM) * (Math.sqrt(Common.MAX_VOLTAGE) / Math.sqrt(robot.batteryVoltageSensor.getVoltage())) * VOLTAGE_SCALER;

        currentPower = feedforwardValue;
        currentPower += velocityController.calculate(currentRPMSmooth);

        if (Math.abs(currentRPMSmooth - shootingRPM) < LOW_PASS_FILTER_RPM_TOLERANCE) {
            currentPower = motorPowerFilter.calculate(currentPower);
            notInToleranceCounter = 0;
        }
        else {
            motorPowerFilter.reset();
            notInToleranceCounter++;
        }

        currentPower = Range.clip(currentPower, feedforwardValue, 1.0);

//        if (robot.shooter.get() == Shooter.ShooterStates.RUNNING) currentPower = motorGroup[0].get();

        for (MotorEx m : motorGroup) {
            if (shootingRPM-currentRPMSmooth>BB_TOLERANCE) m.set(1);
            else m.set(Math.abs(manualPower) > 0 ? manualPower : currentPower);
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

    public boolean isNotStable() {
        return notInToleranceCounter >= OUT_OF_TOLERANCE_LOOPS;
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
