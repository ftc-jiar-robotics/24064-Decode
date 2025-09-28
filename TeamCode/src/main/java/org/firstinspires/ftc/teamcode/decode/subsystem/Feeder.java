package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.BACKWARD;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;

@Configurable
public class Feeder extends Subsystem<Feeder.FeederControl> {
    public static class FeederControl {
        public boolean isShooterReady;
        public boolean isIntaking;
        public FeederStates currentState;

        public FeederControl(boolean isShooterReady, boolean isIntaking, FeederStates currentState) {
            this.isShooterReady = isShooterReady;
            this.isIntaking = isIntaking;
            this.currentState = currentState;
        }
    }

    public final CRServo feederFront;
    public final CRServo feederBack;

    private final ColorSensor colorSensor;

    private float gain = 0; //TODO: change gain

    public enum FeederStates {
        OFF, OUTTAKING, IDLE, RUNNING
    }

    private final FeederControl currentControl = new FeederControl(false, false, FeederStates.OFF);

    public Feeder(HardwareMap hw) {
        feederFront = hw.get(CRServo.class, Common.CFG_NAME_FEEDERFRONT);
        feederBack = hw.get(CRServo.class, Common.CFG_NAME_FEEDERBACK);
        this.colorSensor = new ColorSensor(hw, Common.CFG_NAME_FEEDER_COLORSENSOR, gain);

        feederFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void set(FeederControl control) {
        currentControl.isIntaking = control.isIntaking;
        currentControl.isShooterReady = control.isShooterReady;

        if (control.currentState != FeederStates.OFF && control.currentState != FeederStates.OUTTAKING) {
            if (control.isShooterReady) currentControl.currentState = FeederStates.RUNNING;
            else currentControl.currentState = FeederStates.IDLE;
        } else currentControl.currentState = control.currentState;
    }

    @Override
    public FeederControl get() {
        return currentControl;
    }

    @Override
    public void run() {
        switch (currentControl.currentState) {
            case OFF: {
                feederFront.setPower(0);
                feederBack.setPower(0);
            }
            case OUTTAKING: {
                feederFront.setPower(0.25);
                feederBack.setPower(-0.25);
            }
            case IDLE: {
                feederFront.setPower(-0.25);
                feederBack.setPower(-0.25);
            }
            case RUNNING: {
                feederFront.setPower(-0.25);
                feederBack.setPower(0.25);
            }
        }
    }

    public void printTelemetry() {
        telemetry.addData("current state: ", currentControl.currentState);
        telemetry.addData("intaking boolean: ", currentControl.isIntaking);
        telemetry.addData("shooter ready boolean: ", currentControl.isShooterReady);
    }
}
