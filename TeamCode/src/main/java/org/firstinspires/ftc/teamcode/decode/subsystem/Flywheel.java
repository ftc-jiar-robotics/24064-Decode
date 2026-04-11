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
    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS_CLOSE = new PIDFCoefficients(0.0022, 0, 0.00002, 0.000077);
    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS_FAR = new PIDFCoefficients(0.0029, 0, 0.00002, 0.000077);
    private final SolversPIDF velocityController = new SolversPIDF(FLYWHEEL_PIDF_COEFFICIENTS_CLOSE);
    public static final double GEAR_RATIO = 20.0/20;
    public enum FlyWheelStates {
        IDLE, ARMING, RUNNING
    }
    public static double
            LAUNCH_DELAY = 0.3,
            OUT_OF_TOLERANCE_LOOPS = 3,
            RPM_TOLERANCE = 100,
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
            CLOSE_ADJUSTMENT_RPM = 25, // added onto rpm curve
            FAR_ADJUSTMENT_RPM = 40,
            ROUNDING_POINT = 100;

    private FlyWheelStates targetState = FlyWheelStates.IDLE;

    public static LowPassGains motorPowerGains = new LowPassGains(
            0,
            6);

    private final IIRLowPassFilter motorPowerFilter = new IIRLowPassFilter(motorPowerGains);


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
        CachedMotor shooterMaster = new CachedMotor(hw, NAME_FLYWHEEL_MASTER_MOTOR, Motor.GoBILDA.BARE,ROUNDING_POINT);
        CachedMotor shooterSlave = new CachedMotor(hw, NAME_FLYWHEEL_SLAVE_MOTOR, Motor.GoBILDA.BARE,ROUNDING_POINT);
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

        motorGroup = new CachedMotor[]{shooterMaster, shooterSlave};

       // velocityController.setDerivativeMode(PIDController.DerivativeMode.MEASUREMENT);
//        velocityController.setGains(shootingVelocityGains);
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
                        chooseShootingRPM(robot.shooter.turret.getDistance(calculateTurretPosition(LaunchZone.getInterceptOrClosestPoint(), Math.toDegrees(robot.drivetrain.getHeading()), -Common.TURRET_OFFSET_Y)));
                    } else
                        shootingRPM =  IDLE_RPM;
                }
                velocityController.setSetPoint(shootingRPM);

                break;
            case ARMING:
                chooseShootingRPM(robot.shooter.turret.getDistance(calculateTurretPosition(robot.shooter.getPredictedPose(LAUNCH_DELAY), Math.toDegrees(robot.drivetrain.getHeading()), -Common.TURRET_OFFSET_Y)));
                if (isPIDInTolerance()) targetState = FlyWheelStates.RUNNING;
                break;
            case RUNNING:
                chooseShootingRPM(robot.shooter.turret.getDistance(calculateTurretPosition(robot.shooter.getPredictedPose(LAUNCH_DELAY), Math.toDegrees(robot.drivetrain.getHeading()), -Common.TURRET_OFFSET_Y)));
                break;
        }


        double feedforwardValue = 0;//(shootingRPM/MAX_RPM) * (Math.sqrt(Common.MAX_VOLTAGE) / Math.sqrt(robot.batteryVoltageSensor.getVoltage())) * VOLTAGE_SCALER;

        PIDFCoefficients coefficients = robot.shooter.turret.getDistance() <= SWITCH_PID_DIST ?
                FLYWHEEL_PIDF_COEFFICIENTS_CLOSE :
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
            notInToleranceCounter = 0;
        }
        else {
            motorPowerFilter.reset();
            notInToleranceCounter++;
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
//        dashTelemetry.addData("current power 1 (PERCENTAGE): ", motorGroup[0].getPower());
//        dashTelemetry.addData("current power 2 (PERCENTAGE): ", motorGroup[1].getPower());
        dashTelemetry.addData("current power (PERCENTAGE): ", currentPower);

        dashTelemetry.addData("current pos (TICKS): ", shooterEncoder.getPosition());
        dashTelemetry.addData("target RPM (ROTATIONS PER MINUTE): ", shootingRPM);

    }
}
