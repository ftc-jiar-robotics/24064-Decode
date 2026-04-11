package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.MIN_DISTANCE_FEEDER;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_BACK1_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_BACK2_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_GATE_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_INTAKE_MOTO_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.util.SimpleServoPivot;

@Configurable
public class Feeder extends Subsystem<Feeder.FeederStates> {
    private final SimpleServoPivot feederGate;
    private final CRServo backFeeder1;
    private final CRServo backFeeder2;
    private final MotorEx motor;

    private final DigitalChannel pin0Left, pin0Right;

    private FeederStates currentState = FeederStates.BLOCKING;

    private int
            lastPinState = 0,
            currentPinState = 0;

    public enum FeederStates {
        BLOCKING, RUNNING
    }

    public static double
        BLOCKING_ANGLE = 240,
        RUNNING_ANGLE = 310,
        MAX_PIN_STATE = 7, // default
        CLOSE_SHOOTING_SPEED = .45,
        FAR_SHOOTING_SPEED = .3,
        MID_SHOOTER_SPEED = .4;

    public Feeder(HardwareMap hw) {
        feederGate = new SimpleServoPivot(BLOCKING_ANGLE, RUNNING_ANGLE, SimpleServoPivot.getAxonServo(hw, NAME_FEEDER_GATE_SERVO));
        backFeeder1 = hw.get(CRServo.class, NAME_FEEDER_BACK1_SERVO);
        backFeeder1.setDirection(DcMotorSimple.Direction.REVERSE);
        backFeeder2 = hw.get(CRServo.class, NAME_FEEDER_BACK2_SERVO);
        pin0Left = hw.digitalChannel.get(Common.NAME_FEEDER_LEFT_PIN0);
        pin0Right = hw.digitalChannel.get(Common.NAME_FEEDER_RIGHT_PIN0);
        motor = new MotorEx(hw, NAME_INTAKE_MOTO_MOTOR, Motor.GoBILDA.BARE);
        motor.setInverted(true);
    }

    @Override
    protected void set(FeederStates state) {
        currentState = state;
    }

    public boolean isBallPresent() {
        return currentPinState != 0;
    }

    @Override
    public FeederStates get() {
        return currentState;
    }

    public boolean didShotOccur() {
        currentPinState += pin0Left.getState() || pin0Right.getState() ? 5:-1;
        currentPinState = (int)Range.clip(currentPinState,0,MAX_PIN_STATE);

        if (lastPinState>0 && currentPinState==0) {
            lastPinState = 0;
            return true;
        }

        lastPinState = currentPinState;
        return false;
    }

    @Override
    public void run() {
        feederGate.setActivated(currentState == FeederStates.RUNNING);
        motor.set((Common.MAX_VOLTAGE / robot.batteryVoltageSensor.getVoltage())*(currentState == FeederStates.RUNNING ? (robot.isFar ? FAR_SHOOTING_SPEED : (robot.isMid ? MID_SHOOTER_SPEED : CLOSE_SHOOTING_SPEED)) : (Math.abs(robot.intake.get()) > 0.1 ? .2 : 0)));
        backFeeder1.setPower(currentState == FeederStates.RUNNING ? motor.get() : (Math.abs(robot.intake.get()) > 0.1 ? -1 : 0));
        backFeeder2.setPower(currentState == FeederStates.RUNNING ? motor.get() : (Math.abs(robot.intake.get()) > 0.1 ? -1 : 0));
        feederGate.run();
    }

    public void printTelemetry() {
        feederGate.updateAngles(BLOCKING_ANGLE, RUNNING_ANGLE);

        telemetry.addLine("FEEDER");
        telemetry.addData("current state (ENUM): ", currentState);
        telemetry.addData("is feeder gate activated? (BOOLEAN: ", feederGate.isActivated());
        telemetry.addData("feeder left in range (BOOLEAN): ", pin0Left.getState());
        telemetry.addData("feeder right in range (BOOLEAN): ", pin0Right.getState());
        telemetry.addData("is ball present? (BOOLEAN): ", isBallPresent());

        dashTelemetry.addData("current pin state (INT): ", currentPinState);
        dashTelemetry.addData("last pin state (INT): ", lastPinState);
    }
}
