package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;

@Configurable
public class Feeder extends Subsystem<Feeder.FeederStates> {

    private final CRServo feederFront;
    private final CRServo feederBack;

    private final ColorSensor colorSensor;

    private FeederStates currentState = FeederStates.OFF;

    private float gain = 0; //TODO: change gain

    public enum FeederStates {
        OFF, OUTTAKING, MANUAL, IDLE, RUNNING
    }

    public Feeder(HardwareMap hw) {
        feederFront = hw.get(CRServo.class, Common.NAME_FEEDER_FRONTSERVO);
        feederBack = hw.get(CRServo.class, Common.NAME_FEEDER_BACKSERVO);
        this.colorSensor = new ColorSensor(hw, Common.NAME_FEEDER_COLORSENSOR, gain);

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

    @Override
    public FeederStates get() {
        return currentState;
    }

    @Override
    public void run() {
            switch (currentState) {
                case OFF:
                    feederFront.setPower(0);
                    feederBack.setPower(0);
                    break;
                case OUTTAKING:
                    feederFront.setPower(-1);
                    feederBack.setPower(-1);
                    break;
                case IDLE:
                    feederFront.setPower(1);
                    feederBack.setPower(-0.7);
                    break;
                case RUNNING:
                    feederFront.setPower(1);
                    feederBack.setPower(1);
                    break;
            }
    }

    public void printTelemetry() {
        telemetry.addLine("FEEDER");
        telemetry.addData("current state: ", currentState);
    }
}
