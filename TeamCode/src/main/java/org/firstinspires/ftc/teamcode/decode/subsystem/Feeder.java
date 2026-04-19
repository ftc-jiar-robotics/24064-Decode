package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_BACK1_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_BACK2_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_GATE_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_INTAKE_MOTO_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.util.CachedCRServo;
import org.firstinspires.ftc.teamcode.decode.util.CachedMotor;
import org.firstinspires.ftc.teamcode.decode.util.SimpleServoPivot;

@Configurable
public class Feeder extends Subsystem<Feeder.FeederStates> {
    private final SimpleServoPivot feederGate;
    private final CachedCRServo backFeeder1;
    private final CachedCRServo backFeeder2;
    private final CachedMotor motor;

    private final DigitalChannel pin0Left, pin0Right;

    private FeederStates currentState = FeederStates.BLOCKING;

    public boolean isGateEnabled;

    private int
            lastPinState = 0,
            currentPinState = 0;

    public enum FeederStates {
        BLOCKING, RUNNING
    }

    public static double
        BLOCKING_ANGLE = 180,
        RUNNING_ANGLE = 150,
        MAX_PIN_STATE = 3, // default
        MAX_PIN_STATE_GATE_ENABLED = 17, // default
        CLOSE_SHOOTING_SPEED = .725,
        FAR_SHOOTING_SPEED = .42,
        MID_SHOOTER_SPEED = .55,
        ROUNDING_POINT = 10;

    public Feeder(HardwareMap hw, boolean isAuto) {
        feederGate = new SimpleServoPivot(BLOCKING_ANGLE, RUNNING_ANGLE, SimpleServoPivot.getAxonServo(hw, NAME_FEEDER_GATE_SERVO));
        backFeeder1 = new CachedCRServo(hw, NAME_FEEDER_BACK1_SERVO, ROUNDING_POINT);
        backFeeder2 = new CachedCRServo(hw, NAME_FEEDER_BACK2_SERVO, ROUNDING_POINT);
        backFeeder1.setDirection(DcMotorSimple.Direction.REVERSE);

        pin0Left = hw.digitalChannel.get(Common.NAME_FEEDER_LEFT_PIN0);
        pin0Right = hw.digitalChannel.get(Common.NAME_FEEDER_RIGHT_PIN0);

        motor = new CachedMotor(hw, NAME_INTAKE_MOTO_MOTOR, Motor.GoBILDA.BARE, ROUNDING_POINT);
        motor.setInverted(true);

        isGateEnabled = true;//!isAuto;
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
        currentPinState = (int)Range.clip(currentPinState,0,isGateEnabled ? MAX_PIN_STATE_GATE_ENABLED : MAX_PIN_STATE);

        if (lastPinState>0 && currentPinState==0) {
            lastPinState = 0;
            return true;
        }

        lastPinState = currentPinState;
        return false;
    }

    @Override
    public void run() {
        feederGate.setActivated(currentState == FeederStates.RUNNING || !isGateEnabled);
        motor.set(
                ((Common.MAX_VOLTAGE / robot.getVoltage()) * (
                        currentState == FeederStates.RUNNING ?
                                (robot.isFar ?
                                        FAR_SHOOTING_SPEED :
                                        (robot.isMid ?
                                                MID_SHOOTER_SPEED :
                                                CLOSE_SHOOTING_SPEED
                                        )
                                ) : (
                                        Math.abs(robot.intake.get()) > 0.1 ? .2 : 0
                                )
                ))* (robot.isAuto?.7:1)
        );
        backFeeder1.setPower(currentState == FeederStates.RUNNING ? motor.get() : (Math.abs(robot.intake.get()) > 0.1 ? -1 : 0));
        backFeeder2.setPower(currentState == FeederStates.RUNNING ? motor.get() : (Math.abs(robot.intake.get()) > 0.1 ? -1 : 0));
        feederGate.run();
    }

    public double getSpeed(){
        return motor.get();
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
