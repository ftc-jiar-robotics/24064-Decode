package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_BACK_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_GATE_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;
import org.firstinspires.ftc.teamcode.decode.util.SimpleServoPivot;

@Configurable
public class Feeder extends Subsystem<Feeder.FeederStates> {
    private final SimpleServoPivot feederGate;
    private final CRServo backFeeder;

    private final DigitalChannel pin0Left, pin0Right;

    private FeederStates currentState = FeederStates.BLOCKING;

    public static float GAIN = 1.0f;

    private int
            lastPinState = 0,
            currentPinState = 0;

    public enum FeederStates {
        BLOCKING, RUNNING
    }

    public static double
        BLOCKING_ANGLE = 240,
        RUNNING_ANGLE = 310,
        MAX_PIN_STATE = 7; // default

    public Feeder(HardwareMap hw) {
        feederGate = new SimpleServoPivot(BLOCKING_ANGLE, RUNNING_ANGLE, SimpleServoPivot.getAxonServo(hw, NAME_FEEDER_GATE_SERVO));
        backFeeder = hw.get(CRServo.class, NAME_FEEDER_BACK_SERVO);
        backFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        pin0Left = hw.digitalChannel.get(Common.NAME_FEEDER_LEFT_PIN0);
        pin0Right = hw.digitalChannel.get(Common.NAME_FEEDER_RIGHT_PIN0);
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
        currentPinState += /*pin0Left.getState() || */pin0Right.getState() ? 5:-1;
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
        backFeeder.setPower(currentState == FeederStates.RUNNING ? 1 : (Math.abs(robot.intake.get()) > 0.1 ? -1 : 0));

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
