package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.graph;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;

@Configurable
@Config
public class Flywheel extends Subsystem<Flywheel.FlyWheelStates> {
    private final DcMotorEx shooterMaster, shooterSlave;
    private final DcMotorEx[] motorGroup;

    private final Motor.Encoder shooterEncoder;

    private final PIDController velocityController = new PIDController();

    public static PIDGains shootingVelocityGains = new PIDGains(
            0.000375,
            0.00001,
            0.00015,
            Double.POSITIVE_INFINITY
    );

    public static PIDGains holdingVelocityGains = new PIDGains(
            0.0000004,
            0.000001,
            0.000003,
            Double.POSITIVE_INFINITY
    );

    public enum FlyWheelStates {
        OFF, IDLE, ARMING, RUNNING
    }

    public static double
            currentSpikeThreshold = 2.5, // TODO tune in AMPS
            timeDropPeriod = 0.5, // TODO tune in SECONDS of how long current should drop
            rpmChangeDistance = 75.5,
            closeRPM = 3890,
            farRPM = 4000;

    private FlyWheelStates targetState = FlyWheelStates.OFF;

    private boolean inCurrentSpikeTimer = false;
    private double
            rpmTolerance = 50,
            currentRPM = 0.0,
            flywheelCurrent = 0.0,
            lastCurrentReading = 0.0,
            shootingRPM = 4000,
            currentPower = 0,
            calculatedPower = 0,
            currentSpikeStartTime = 0;

    public Flywheel(HardwareMap hw) {
        this.shooterMaster = (DcMotorEx) hw.get(DcMotor.class, "shooterMaster");
        this.shooterSlave = (DcMotorEx) hw.get(DcMotor.class, "shooterSlave");
        MotorEx dummy = new MotorEx(hw, "left front", Motor.GoBILDA.BARE);

        shooterSlave.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMaster.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterEncoder = dummy.encoder;

        motorGroup = new DcMotorEx[]{shooterMaster, shooterSlave};

        velocityController.setGains(shootingVelocityGains);
    }

    @Override
    public void set(FlyWheelStates f) {
        targetState = f;
    }

    @Override
    public FlyWheelStates get() {
        return targetState;
    }

    public boolean inCurrentSpikeTimer() {
        return inCurrentSpikeTimer;
    }


    public boolean isPIDInTolerance() {
        return velocityController.isPositionInTolerance(new State(currentRPM, 0, 0, 0), rpmTolerance);
    }

    public boolean didCurrentSpike() {
        // if current has spiked and we're in tolerance and we're not in a timer(start time + time period > curr time
        if (flywheelCurrent > currentSpikeThreshold && !inCurrentSpikeTimer) {
            currentSpikeStartTime = (double) System.nanoTime() / 1E9;
            inCurrentSpikeTimer = true;
            return true;
        }

        if (currentSpikeStartTime + timeDropPeriod < (double) System.nanoTime() / 1E9 && inCurrentSpikeTimer) {
            inCurrentSpikeTimer = false;
            return false;
        }

        return false;
    }

    @Override
    public void run() {
        currentRPM = shooterEncoder.getCorrectedVelocity() * 60.0 / 28.0;
        if (currentRPM > 10000) currentRPM = 0;

        flywheelCurrent = (motorGroup[0].getCurrent(CurrentUnit.AMPS) + motorGroup[1].getCurrent(CurrentUnit.AMPS))/2;

        switch (targetState) {
            case OFF:
                velocityController.setTarget(new State(0, 0, 0, 0));
                break;
            case IDLE:
                velocityController.setGains(holdingVelocityGains);
                velocityController.setTarget(new State(2200, 0, 0, 0));
                break;
            case ARMING:
                velocityController.setGains(shootingVelocityGains);
                velocityController.setTarget(new State(shootingRPM, 0, 0, 0));

                if (isPIDInTolerance()) {
                    targetState = FlyWheelStates.RUNNING;
                }
                break;
            case RUNNING:
                velocityController.setTarget(new State(shootingRPM, 0, 0, 0));
                break;
        }

        if (targetState == FlyWheelStates.ARMING || targetState == FlyWheelStates.RUNNING) {
            calculatedPower = Math.pow(velocityController.calculate(new State(currentRPM, 0, 0, 0)), 3);
        } else calculatedPower = velocityController.calculate(new State(currentRPM, 0, 0, 0));

        currentPower += calculatedPower;
        currentPower = Range.clip(currentPower, 0.0, 1.0);

        for (DcMotorEx m : motorGroup) m.setPower(currentPower);

        if (isPIDInTolerance()) velocityController.reset();
    }

    public void printTelemetry() {
        telemetry.addLine("FLYWHEEL");
        telemetry.addData("current state: ", get());
        telemetry.addData("target state: ", targetState);
        telemetry.addData("current RPM: ", currentRPM);
        telemetry.addData("initial current: ", lastCurrentReading);
        graph.addData("current: ", flywheelCurrent);
        graph.addData("current RPM: ", currentRPM);
        graph.addData("current pos: ", shooterEncoder.getPosition());
        graph.addData("target RPM: ", 4800);
        telemetry.addData("is PID in tolerance: ", isPIDInTolerance());

        dashTelemetry.addData("current: ", flywheelCurrent);
        dashTelemetry.addData("current RPM: ", currentRPM);
        dashTelemetry.addData("calculated power: ", calculatedPower);
        dashTelemetry.addData("current power: ", currentPower);
        dashTelemetry.addData("is timer on: ", inCurrentSpikeTimer);
        dashTelemetry.addData("current pos: ", shooterEncoder.getPosition());
        dashTelemetry.addData("target RPM: ", 4000);
        dashTelemetry.addData("target RPM (idle): ", 2200);

    }
}
