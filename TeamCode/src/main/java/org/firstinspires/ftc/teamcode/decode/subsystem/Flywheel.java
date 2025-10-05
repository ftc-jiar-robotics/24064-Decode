package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.graph;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;

@Configurable
public class Flywheel extends Subsystem<Flywheel.FlyWheelStates> {
    private final DcMotorEx shooterMaster, shooterSlave;
    public final DcMotorEx[] motorGroup;

    private final PIDController velocityController = new PIDController();

    public static PIDGains velocityPidGains = new PIDGains(
            0.005,
            0.002,
            0.0001,
            Double.POSITIVE_INFINITY
    );

    public enum FlyWheelStates {
        OFF, IDLE, ARMING, RUNNING;
    }

    private final ElapsedTime currentSpikeCheckTimer = new ElapsedTime();

    public static double
            currentSpikeThreshold = 0.5, // TODO tune in AMPS
            timeDropPeriod = 0.2; // TODO tune in SECONDS of how long current should drop

    private FlyWheelStates targetState = FlyWheelStates.OFF;

    private double
            currentPower = 0.0,
            flywheelCurrent = 0.0,
            initialCurrentReading = 0.0;

    public Flywheel(HardwareMap hw) {
        this.shooterMaster = (DcMotorEx) hw.get(DcMotor.class, "shooterMaster");
        this.shooterSlave = (DcMotorEx) hw.get(DcMotor.class, "shooterSlave");

        shooterSlave.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMaster.setDirection(DcMotorSimple.Direction.REVERSE);


        motorGroup = new DcMotorEx[]{shooterMaster, shooterSlave};

        velocityController.setGains(velocityPidGains);
    }

    @Override
    public void set(FlyWheelStates f) {
        targetState = f;
    }

    @Override
    public FlyWheelStates get() {
        return targetState;
    }

    public FlyWheelStates getCurrentState() {
        if (currentPower == 0)
            return FlyWheelStates.OFF;
        else if (Math.abs(currentPower) <= .5)
            return FlyWheelStates.IDLE;
        else if (Math.abs(currentPower) <= .8)
            return FlyWheelStates.ARMING;
        else
            return FlyWheelStates.RUNNING;
    }

    public boolean isPIDInTolerance() {
        return velocityController.isPositionInTolerance(new State(currentPower, 0, 0, 0), 0.05);
    }

    public boolean didCurrentDrop() {
        if (flywheelCurrent > initialCurrentReading + currentSpikeThreshold) currentSpikeCheckTimer.startTime();
        if ((currentSpikeCheckTimer.seconds() >= timeDropPeriod && currentSpikeCheckTimer.seconds() <= timeDropPeriod + 0.1) && flywheelCurrent < initialCurrentReading - currentSpikeThreshold) {
            currentSpikeCheckTimer.reset();

            return true;
        } else if (currentSpikeCheckTimer.seconds() > timeDropPeriod + 0.1) currentSpikeCheckTimer.reset();

        return false;
    }

    @Override
    public void run() {
        currentPower = shooterMaster.getPower();

        flywheelCurrent = (motorGroup[0].getCurrent(CurrentUnit.AMPS) + motorGroup[1].getCurrent(CurrentUnit.AMPS))/2;

        switch (targetState) {
            case OFF:
                velocityController.setTarget(new State(0, 0, 0, 0));
                break;
            case IDLE:
                velocityController.setTarget(new State(0.3, 0, 0, 0));
                break;
            case ARMING:
                velocityController.setTarget(new State(1, 0, 0, 0));

                if (isPIDInTolerance()) {
                    initialCurrentReading = (motorGroup[0].getCurrent(CurrentUnit.AMPS) + motorGroup[1].getCurrent(CurrentUnit.AMPS))/2;
                    targetState = FlyWheelStates.RUNNING;
                }
                break;
            case RUNNING:
                velocityController.setTarget(new State(1, 0, 0, 0));
                break;
        }

        for (DcMotorEx m : motorGroup) m.setPower(velocityController.calculate(new State(currentPower, 0, 0, 0)));
    }

    public void printTelemetry() {
        telemetry.addLine("FLYWHEEL");
        telemetry.addData("current state: ", getCurrentState());
        telemetry.addData("target state: ", targetState);
        graph.addData("current: ", flywheelCurrent);
    }
}
