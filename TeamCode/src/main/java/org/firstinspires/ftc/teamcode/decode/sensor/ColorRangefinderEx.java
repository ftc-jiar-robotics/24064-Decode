package org.firstinspires.ftc.teamcode.decode.sensor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ColorRangefinderEx {
    public DigitalChannel
            pin0,
            pin1;

    public AnalogInput analogPin0;

    public enum SampleColor {
        YELLOW,
        BLUE,
        RED,
        NOTHING
    }

    public enum Modes {
        ANALOG,
        DIGITAL

    }

    private SampleColor rawReading = SampleColor.NOTHING;
    private final Modes currentMode;

    public ColorRangefinderEx(HardwareMap hardwareMap, Modes mode) {
        currentMode = mode;

        if (currentMode == Modes.DIGITAL) {
            pin0 = hardwareMap.digitalChannel.get("digital0");
            pin1 = hardwareMap.digitalChannel.get("digital1");
        } else {
            analogPin0 = hardwareMap.analogInput.get("analog0");
        }

    }

    public SampleColor convertToEnum() {
        if (pin0.getState()) {
            if (pin1.getState()) return SampleColor.YELLOW;
            if (!pin1.getState()) return SampleColor.BLUE;
        }

        if (pin1.getState()) {
            if (!pin0.getState()) return SampleColor.RED;
        }
        return SampleColor.NOTHING;
    }

    public double getAnalogHue() {
        return analogPin0.getVoltage() / 3.3 * 360;
    }

    public SampleColor getRawReading() {
        return rawReading;
    }

    public Modes getMode() {
        return currentMode;
    }

    public SampleColor run() {
        if (currentMode == Modes.DIGITAL) rawReading = convertToEnum();
        else rawReading = SampleColor.NOTHING;
        return rawReading;
    }
}
