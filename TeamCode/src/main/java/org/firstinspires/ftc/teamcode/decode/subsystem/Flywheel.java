package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;

@Configurable
public class Flywheel extends Subsystem<Flywheel.FlyWheelStates> {
    private final MotorEx shooterMaster, shooterSlave;
    public final MotorEx[] motorGroup;

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

    private FlyWheelStates targetState = FlyWheelStates.OFF;
    private double currentPower = 0;

    public Flywheel(HardwareMap hw) {
        this.shooterMaster = new MotorEx(hw, "shooterMaster", Motor.GoBILDA.BARE);
        this.shooterSlave = new MotorEx(hw, "shooterSlave", Motor.GoBILDA.BARE);

        shooterSlave.setInverted(true);
        shooterMaster.setInverted(true);


        motorGroup = new MotorEx[]{shooterMaster, shooterSlave};

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

    @Override
    public void run() {
        currentPower = shooterMaster.get();

        switch (targetState) {
            case OFF:
                velocityController.setTarget(new State(0, 0, 0, 0));
            case IDLE:
                velocityController.setTarget(new State(-0.3, 0, 0, 0));
            case ARMING:
                velocityController.setTarget(new State(-0.65, 0, 0, 0));
            case RUNNING:
                velocityController.setTarget(new State(-1, 0, 0, 0));
        }

        for (MotorEx m : motorGroup) m.set(velocityController.calculate(new State(currentPower, 0, 0, 0)));
    }

    public void printTelemetry() {
        telemetry.addData("current state: ", getCurrentState());
        telemetry.addData("target state: ", targetState);
    }
}
