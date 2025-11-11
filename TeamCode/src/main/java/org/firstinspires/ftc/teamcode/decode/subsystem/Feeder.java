package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;

@Configurable
public class Feeder extends Subsystem<Feeder.FeederStates> {
    private final CRServo feederFront;
    private final CRServo feederBack;
    private final ColorSensor colorSensor;

    private FeederStates currentState = FeederStates.OFF;

    public static float GAIN = 1.0f;


    public enum FeederStates {
        OFF, OUTTAKING, IDLE, RUNNING, MANUAL
    }
    public static double[][] feederPowers = {{0, 0}, {-1, -1}, {0.2 , -1} ,{1 , 1}};

    public Feeder(HardwareMap hw) {
        feederFront = hw.get(CRServo.class, Common.NAME_FEEDER_FRONT_SERVO);
        feederBack = hw.get(CRServo.class, Common.NAME_FEEDER_BACK_SERVO);
        this.colorSensor = new ColorSensor(hw, Common.NAME_FEEDER_COLOR_SENSOR, GAIN);

        feederFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void set(double powerFront, double powerBack) {
        if (!isLocked() && powerFront != 0 && powerBack != 0) {
            currentState = FeederStates.MANUAL;
            feederFront.setPower(powerFront);
            feederBack.setPower(powerBack);
        }
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

    @Override
    public void run() {
        double[] ordinal = feederPowers[currentState.ordinal()];
        feederFront.setPower(ordinal[0]);
        feederBack.setPower(ordinal[1]);
        if ((LoopUtil.getLoops() & Common.COLOR_SENSOR_UPDATE_LOOPS) == 0)
            colorSensor.update();
    }

    public void printTelemetry() {
        telemetry.addLine("FEEDER");
        telemetry.addData("current state (ENUM): ", currentState);
        telemetry.addData("feeder front power (PERCENTAGE): ", feederFront.getPower());
        telemetry.addData("feeder back power (PERCENTAGE): ", feederBack.getPower());
        telemetry.addData("curr color (ENUM): ", getColor());
        telemetry.addData("curr color (HSV): ", colorSensor.hsv);
    }
}
