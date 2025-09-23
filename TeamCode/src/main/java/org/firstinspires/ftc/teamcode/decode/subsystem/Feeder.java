package org.firstinspires.ftc.teamcode.decode.subsystem;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;

public class Feeder extends Subsystem<Feeder.FeederControl> {
    class FeederControl {
        boolean isShooterReady;
        boolean isIntaking;
        
    }

    private final CRServo feederFront;
    private final CRServo feederBack;

    private final ColorSensor colorSensor;


    private float gain = 0; //TODO: change gain

    public enum FeederStates {
        OFF, IDLE, RUNNING;
    }

    private FeederStates currentState = FeederStates.OFF;

    public Feeder(HardwareMap hw) {
        feederFront = hw.get(CRServo.class, Common.CFG_NAME_FEEDERFRONT);
        feederBack = hw.get(CRServo.class, Common.CFG_NAME_FEEDERBACK);
        this.colorSensor = new ColorSensor(hw, Common.CFG_NAME_FEEDER_COLORSENSOR, gain);

    }

    @Override
    protected void set(FeederControl control) {

    }

    @Override
    public FeederControl get() {
        return null;
    }

    @Override
    public void run() {

    }
}
