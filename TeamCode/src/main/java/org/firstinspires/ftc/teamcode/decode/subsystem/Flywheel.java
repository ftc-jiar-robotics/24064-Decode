package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FLYWHEEL_MASTER_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FLYWHEEL_SLAVE_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFlywheelManual;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
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
    private final MotorEx shooterMaster, shooterSlave;
    private final MotorEx[] motorGroup;
    private final Differentiator differentiator = new Differentiator();

    private final Motor.Encoder shooterEncoder;

    private final PIDController velocityController = new PIDController();

    public static PIDGains shootingVelocityGains = new PIDGains(
            0.0005,
            0.0,
            0.00025,
            Double.POSITIVE_INFINITY
    );

    private final FIRLowPassFilter rpmFilter = new FIRLowPassFilter();
    public static MovingAverageGains rpmDerivAverageFilterGains = new MovingAverageGains(
            3
    );

    private final MovingAverageFilter rpmDerivAverageFilter = new MovingAverageFilter(rpmDerivAverageFilterGains);

    public static LowPassGains rpmFilterGains = new LowPassGains(
            0.5,
            10
    );

    public enum FlyWheelStates {
        IDLE, ARMING, RUNNING
    }

    public static double
            RPM_DERIVATIVE_DROP = -1500, // deacceleration
            TIME_DROP_PERIOD = 0.5,
            RPM_TOLERANCE = 100,
            SMOOTH_RPM_GAIN = 0.8,
            SUPER_SMOOTH_RPM_GAIN = 0.85,
            DERIV_TOLERANCE = 200,
            MOTOR_RPM_SETTLE_TIME_SHOOT = 0.95,
            MOTOR_RPM_SETTLE_TIME_IDLE = 1.25 ,
            IDLE_RPM = 1200,
            MAX_RPM = 4800,
            VOLTAGE_SCALER = 0.99;

    public static int[] lutDistances = {0, 81, 115, 160 , 180};
    public static int[] lutRPM = {2800, 3300, 3900, 4150, 4800};

    private FlyWheelStates targetState = FlyWheelStates.IDLE;

    public static int TOLERANCE_SAMPLE_COUNT = 10;

    private boolean inCurrentRPMSpike = false;
    private double
            currentRPM = 0.0,
            currentRPMSmooth = 0.0,
            currentRPMSuperSmooth = 0.0,
            currentRPMDerivative = 0.0,
            manualPower = 0.0,
            shootingRPM = 4000,
            settleTime = MOTOR_RPM_SETTLE_TIME_IDLE,
            lastTarget = shootingRPM,
            currentPower = 0,
            startPIDDisable = 0,
            calculatedPower = 0,
            currentRPMSpikeTime = 0;

    public Flywheel(HardwareMap hw) {
        this.shooterMaster = new MotorEx(hw, NAME_FLYWHEEL_MASTER_MOTOR, Motor.GoBILDA.BARE);
        this.shooterSlave = new MotorEx(hw, NAME_FLYWHEEL_SLAVE_MOTOR, Motor.GoBILDA.BARE);
        MotorEx dummy = new MotorEx(hw, "left front", Motor.GoBILDA.BARE);

        shooterSlave.setInverted(true);
        shooterMaster.setInverted(true);

        shooterEncoder = dummy.encoder;

        motorGroup = new MotorEx[]{shooterMaster, shooterSlave};

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

    public void setManualPower(double power) {
        manualPower = power;
    }

    public boolean isPIDInTolerance() {
        return (velocityController.isInTolerance(new State(currentRPMSmooth, 0, 0, 0), RPM_TOLERANCE, DERIV_TOLERANCE));
    }

    public boolean didRPMSpike() {
        currentRPMDerivative = rpmDerivAverageFilter.calculate(differentiator.getDerivative(currentRPMSuperSmooth));

        // if current has spiked and we're in tolerance and we're not in a timer(start time + time period > curr time
        if (currentRPMDerivative < RPM_DERIVATIVE_DROP && !inCurrentRPMSpike) {
            currentRPMSpikeTime = (double) System.nanoTime() / 1E9;
            inCurrentRPMSpike = true;
            startPIDDisable = (double) System.nanoTime() / 1E9;
            settleTime = MOTOR_RPM_SETTLE_TIME_SHOOT;
            return true;
        }

        if ((currentRPMSpikeTime + TIME_DROP_PERIOD < (double) System.nanoTime() / 1E9 || currentRPMDerivative > RPM_DERIVATIVE_DROP) && inCurrentRPMSpike) {
            inCurrentRPMSpike = false;
            return false;
        }

        return false;
    }

    @Override
    public void run() {
        currentRPM = (shooterEncoder.getCorrectedVelocity() * 60.0 / 28.0);
        currentRPMSmooth = (SMOOTH_RPM_GAIN * currentRPMSmooth) + (1 - SMOOTH_RPM_GAIN) * currentRPM;
        currentRPMSuperSmooth = (SUPER_SMOOTH_RPM_GAIN * currentRPMSuperSmooth) + (1 - SUPER_SMOOTH_RPM_GAIN) * currentRPM;
        if (currentRPM > 10000) currentRPM = 0;
        if (currentRPMSmooth > 10000) currentRPMSmooth = 0;
        if (currentRPMSuperSmooth > 10000) currentRPMSuperSmooth = 0;

        switch (targetState) {
            case IDLE:
                if (!isFlywheelManual) shootingRPM = IDLE_RPM;
                velocityController.setTarget(new State(shootingRPM, 0, 0, 0));

                settleTime = MOTOR_RPM_SETTLE_TIME_IDLE;
                break;
            case ARMING:
                velocityController.setGains(shootingVelocityGains);
                chooseShootingRPM(robot.shooter.turret.getDistance());

                if (isPIDInTolerance()) {
                    targetState = FlyWheelStates.RUNNING;
                }
                break;
            case RUNNING:
                chooseShootingRPM(robot.shooter.turret.getDistance());
                break;
        }


        currentPower = (shootingRPM/MAX_RPM) * (Math.sqrt(Common.MAX_VOLTAGE) / Math.sqrt(robot.batteryVoltageSensor.getVoltage())) * VOLTAGE_SCALER;
        currentPower += velocityController.calculate(new State(currentRPMSmooth, 0, 0, 0));

        currentPower = Range.clip(currentPower, 0.0, 1.0);

        for (MotorEx m : motorGroup) m.set(Math.abs(manualPower) > 0 ? manualPower : currentPower);

        if (isPIDInTolerance() && robot.shooter.getQueuedShots() <= 0) velocityController.reset();
    }

    public void incrementFlywheelRPM(double RPM, boolean isIncrementing) {
        if (isIncrementing) shootingRPM += RPM;
        else shootingRPM -= RPM;

        velocityController.setTarget(new State(shootingRPM, 0, 0, 0));
    }

    private void chooseShootingRPM(double distance) {
//        shootingRPM = lutRPM[0];
//        for (int i = 0; i < lutDistances.length; i++) {
//            if (Common.robot.shooter.turret.getDistance() >= lutDistances[i]) shootingRPM = lutRPM[i];
//        }
        if (!isFlywheelManual) {
            shootingRPM = 1657.1038201234544*(1) + 11.613561597353288*(distance);
                    velocityController.setTarget(new State(shootingRPM, 0, 0, 0));
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
        dashTelemetry.addData("current RPM Super Smooth (ROTATIONS PER MINUTE): ", currentRPMSuperSmooth);
        dashTelemetry.addData("current RPM Derivative (ROTATIONS PER MINUTE): ", currentRPMDerivative);
        dashTelemetry.addData("calculated power (PERCENTAGE): ", calculatedPower);
        dashTelemetry.addData("current power (PERCENTAGE): ", currentPower);
        dashTelemetry.addData("is timer on (BOOLEAN): ", inCurrentRPMSpike);
        dashTelemetry.addData("current pos (TICKS): ", shooterEncoder.getPosition());
        dashTelemetry.addData("target RPM (ROTATIONS PER MINUTE): ", shootingRPM);
        dashTelemetry.addData("current settle time (LOOPS): ", settleTime);

    }
}
