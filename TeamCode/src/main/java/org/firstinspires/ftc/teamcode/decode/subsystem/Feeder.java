package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;

@Configurable
public class Feeder extends Subsystem<Feeder.FeederStates> {
    private final CRServo feederFront;
    private final CRServo feederBack;
    private final ColorSensor colorSensor;

    private FeederStates currentState = FeederStates.OFF;

    private float gain = 1; //TODO: change gain

    public enum ArtifactColor {
        GREEN, PURPLE, NONE
    }

    public static HSV
            GREEN_MIN = new HSV(130, 0.5, 0.01),
            GREEN_MAX = new HSV(160, 1, 0.2),
            PURPLE_MIN = new HSV(205, 0.55, 0.01),
            PURPLE_MAX = new HSV(225, 1, 0.35);

    public enum FeederStates {
        OFF, OUTTAKING, IDLE, RUNNING, MANUAL
    }
    public static double[][] feederPowers = {{0, 0}, {-1, -1}, {0.2 , -1} ,{1 , 1}};

    public Feeder(HardwareMap hw) {
        feederFront = hw.get(CRServo.class, Common.NAME_FEEDER_FRONT_SERVO);
        feederBack = hw.get(CRServo.class, Common.NAME_FEEDER_BACK_SERVO);
        this.colorSensor = new ColorSensor(hw, Common.NAME_FEEDER_COLOR_SENSOR, gain);

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

    public ArtifactColor getColor() {
        if (colorSensor.hsv.inRange(GREEN_MIN, GREEN_MAX)) return ArtifactColor.GREEN;
        else if (colorSensor.hsv.inRange(PURPLE_MIN, PURPLE_MAX)) return ArtifactColor.PURPLE;
        else return ArtifactColor.NONE;
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
