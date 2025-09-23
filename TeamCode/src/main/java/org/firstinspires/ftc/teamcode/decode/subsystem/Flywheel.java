package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;

public class Flywheel extends Subsystem<Flywheel.FlyWheelStates> {
    private final MotorEx shooterMaster, shooterSlave;
    private final MotorEx[] motorGroup;

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

    private FlyWheelStates targetState;
    private double currentPower = 0;

    public Flywheel(HardwareMap hw) {
        this.shooterMaster = new MotorEx(hw, "shooterMaster", Motor.GoBILDA.BARE);
        this.shooterSlave = new MotorEx(hw, "shooterMaster", Motor.GoBILDA.BARE);

        motorGroup = new MotorEx[]{shooterMaster, shooterSlave};

        velocityController.setGains(velocityPidGains);
    }

    public void set(FlyWheelStates f) {
        targetState = f;
    }

    public FlyWheelStates get() {
        return targetState;
    }

    public FlyWheelStates getCurrentState() {
        if (currentPower == 0)
            return FlyWheelStates.OFF;
        else if (currentPower <= .5)
            return FlyWheelStates.IDLE;
        else if (currentPower <= .8)
            return FlyWheelStates.ARMING;
        else
            return FlyWheelStates.RUNNING;
    }

    @Override
    public void run() {
        currentPower = shooterMaster.get();

        switch (targetState) {
            case OFF:
                velocityController.setTarget(new State(0, 0, 0, 0));
            case IDLE:
                velocityController.setTarget(new State(0.3, 0, 0, 0));
            case ARMING:
                velocityController.setTarget(new State(0.65, 0, 0, 0));
            case RUNNING:
                velocityController.setTarget(new State(1, 0, 0, 0));
        }

        for (MotorEx m : motorGroup) m.set(velocityController.calculate(new State(currentPower, 0, 0, 0)));
    }
}
