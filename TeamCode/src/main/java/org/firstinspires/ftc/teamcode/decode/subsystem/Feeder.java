package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;
import org.firstinspires.ftc.teamcode.decode.util.DistanceSensorEx;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;

@Configurable
public class Feeder extends Subsystem<Feeder.FeederStates> {
    private final CRServo feederFront;
    private final CRServo feederBack;
    private final ColorSensor colorSensor;

    private final DistanceSensorEx leftDistanceSensor, rightDistanceSensor;

    private FeederStates currentState = FeederStates.OFF;

    public static float GAIN = 1.0f;

    // TODO tune
    public static double
            MIN_DISTANCE = 0.075, // INCHES
            MAX_DISTANCE = 10; // INCHES

    private double
            leftDistance = 0,
            rightDistance = 0;

    public enum FeederStates {
        OFF, OUTTAKING, IDLE, RUNNING, MANUAL
    }
    public static double[][] feederPowers = {{0, 0}, {-1, -1}, {0.2 , -1} ,{1 , 1}};

    public Feeder(HardwareMap hw) {
        feederFront = hw.get(CRServo.class, Common.NAME_FEEDER_FRONT_SERVO);
        feederBack = hw.get(CRServo.class, Common.NAME_FEEDER_BACK_SERVO);
        colorSensor = new ColorSensor(hw, Common.NAME_FEEDER_COLOR_SENSOR, GAIN);
        leftDistanceSensor = new DistanceSensorEx(hw.get(DistanceSensor.class, Common.NAME_FEEDER_LEFT_DISTANCE_SENSOR));
        rightDistanceSensor = new DistanceSensorEx(hw.get(DistanceSensor.class, Common.NAME_FEEDER_RIGHT_DISTANCE_SENSOR));

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

    public boolean isInLeftRange() {
        return leftDistanceSensor.calculateDistance() < MAX_DISTANCE && leftDistanceSensor.calculateDistance() > MIN_DISTANCE;
    }

    public boolean isInRightRange() {
        return rightDistanceSensor.calculateDistance() < MAX_DISTANCE && rightDistanceSensor.calculateDistance() > MIN_DISTANCE;
    }

    @Override
    public void run() {
        double[] ordinal = feederPowers[currentState.ordinal()];
        feederFront.setPower(ordinal[0]);
        feederBack.setPower(ordinal[1]);
        if ((LoopUtil.getLoops() & Common.COLOR_SENSOR_UPDATE_LOOPS) == 0)
            colorSensor.update();

        if ((LoopUtil.getLoops() & Common.DISTANCE_SENSOR_UPDATE_LOOPS) == 0) {
            leftDistance = leftDistanceSensor.calculateDistance();
            rightDistance = rightDistanceSensor.calculateDistance();
        }
    }

    public void printTelemetry() {
        telemetry.addLine("FEEDER");
        telemetry.addData("current state (ENUM): ", currentState);
        telemetry.addData("feeder front power (PERCENTAGE): ", feederFront.getPower());
        telemetry.addData("feeder back power (PERCENTAGE): ", feederBack.getPower());
        telemetry.addData("feeder left distance (INCHES): ", leftDistance);
        telemetry.addData("feeder right distance (INCHES): ", rightDistance);
        telemetry.addData("curr color (ENUM): ", getColor());
        telemetry.addData("curr color (HSV): ", colorSensor.hsv);
    }
}
