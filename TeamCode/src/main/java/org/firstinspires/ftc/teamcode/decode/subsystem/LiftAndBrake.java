package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.util.SimpleServoPivot;

public class LiftAndBrake extends Subsystem<LiftAndBrake.LiftConfig> {
    private final SimpleServoPivot ptoEngager;
    private final CRServo parkingBrakeMaster;
    private final CRServo parkingBrakeSlave;

    public static double
        INITIAL_ANGLE_PTO = 0,
        ENGAGED_ANGLE_PTO = 10;

    public enum LiftStates {
        PARKING_BRAKE,
        LIFT,
        FINISHED
    }

    public static class LiftConfig {
        LiftStates currentState;
        private final double targetBrakeTime;

        private double startTime = 0;

        LiftConfig(LiftStates l, double t) {
            this.currentState = l;
            this.targetBrakeTime = t;
        }

        public void startTimer() {
            startTime = System.nanoTime() / 1e9;
        }

        public boolean isTimerRunning() {
            return (System.nanoTime() / 1e9) - startTime < targetBrakeTime;
        }
    }

    private LiftConfig currentConfig = new LiftConfig(LiftStates.PARKING_BRAKE, 0);

    public LiftAndBrake(HardwareMap hw) {
        this.ptoEngager = new SimpleServoPivot(INITIAL_ANGLE_PTO, ENGAGED_ANGLE_PTO, SimpleServoPivot.getGoBildaServo(hw, Common.NAME_PTO_SERVO));
        this.parkingBrakeMaster = new CRServo(hw, Common.NAME_BRAKE_MASTER_SERVO);
        this.parkingBrakeSlave = new CRServo(hw, Common.NAME_BRAKE_SLAVE_SERVO);

        parkingBrakeSlave.setInverted(true);
    }

    @Override
    protected void set(LiftConfig l) {
        currentConfig = l;
        currentConfig.startTimer();
    }

    public boolean getIsLiftStarted() {
        return currentConfig.currentState == LiftStates.LIFT;
    }

    @Override
    public LiftConfig get() {
        return currentConfig;
    }

    @Override
    public void run() {
        if (currentConfig.currentState != LiftStates.FINISHED && !currentConfig.isTimerRunning()) {
            currentConfig.currentState = LiftStates.FINISHED;
        }
        if (currentConfig.currentState == LiftStates.PARKING_BRAKE) {
            parkingBrakeMaster.set(1);
            parkingBrakeSlave.set(1);
        } else {
            ptoEngager.run();
        }
    }

    @Override
    public void printTelemetry() {
        telemetry.addLine("LIFT AND BRAKE");
        telemetry.addData("CURRENT STATE: ", currentConfig.currentState);
        telemetry.addData("CURRENT BRAKE TIME: ", currentConfig.targetBrakeTime);
    }
}
