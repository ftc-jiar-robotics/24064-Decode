package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_GATE_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;
import org.firstinspires.ftc.teamcode.decode.util.SimpleServoPivot;

@Configurable
public class Feeder extends Subsystem<Feeder.FeederStates> {
    private final SimpleServoPivot feederGate;
    private final ColorSensor colorSensor;

    private final DigitalChannel pin0Left, pin0Right;

    private FeederStates currentState = FeederStates.RUNNING;

    public static float GAIN = 1.0f;

    private boolean
            lastPinState = false,
            currentPinState = false;

    public enum FeederStates {
        BLOCKING, RUNNING
    }

    public static double
        BLOCKING_ANGLE = 0,
        RUNNING_ANGLE = 0; // default

    public Feeder(HardwareMap hw) {
        feederGate = new SimpleServoPivot(RUNNING_ANGLE, BLOCKING_ANGLE, SimpleServoPivot.getAxonServo(hw, NAME_FEEDER_GATE_SERVO));
        colorSensor = new ColorSensor(hw, Common.NAME_FEEDER_COLOR_SENSOR, GAIN);
        pin0Left = hw.digitalChannel.get(Common.NAME_FEEDER_LEFT_PIN0);
        pin0Right = hw.digitalChannel.get(Common.NAME_FEEDER_RIGHT_PIN0);
    }

    @Override
    protected void set(FeederStates state) {
        currentState = state;
    }

    public Robot.ArtifactColor getColor() {
        return Robot.getColor(colorSensor, true);
    }

    @Override
    public FeederStates get() {
        return currentState;
    }

    public boolean didShotOccur() {
        currentPinState = pin0Left.getState() || pin0Right.getState();

        if (lastPinState && !currentPinState) {
            lastPinState = false;
            return true;
        }

        lastPinState = currentPinState;
        return false;
    }

    @Override
    public void run() {
        feederGate.setActivated(currentState == FeederStates.RUNNING);

        if ((LoopUtil.getLoops() & Common.COLOR_SENSOR_UPDATE_LOOPS) == 0)
            colorSensor.update();

        feederGate.run();
    }

    public void printTelemetry() {
        feederGate.updateAngles(RUNNING_ANGLE, BLOCKING_ANGLE);

        telemetry.addLine("FEEDER");
        telemetry.addData("current state (ENUM): ", currentState);
        telemetry.addData("is feeder gate activated? (BOOLEAN: ", feederGate.isActivated());
        telemetry.addData("feeder left in range (BOOLEAN): ", pin0Left.getState());
        telemetry.addData("feeder right in range (BOOLEAN): ", pin0Right.getState());
        telemetry.addData("curr color (ENUM): ", getColor());
        telemetry.addData("curr color (HSV): ", colorSensor.hsv);

        dashTelemetry.addData("current pin state (INT): ", currentPinState ? 1 : 0);
        dashTelemetry.addData("last pin state (INT): ", lastPinState ? 1 : 0);
    }
}
