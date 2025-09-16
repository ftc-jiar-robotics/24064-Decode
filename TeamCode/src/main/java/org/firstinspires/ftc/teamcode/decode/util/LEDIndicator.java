package org.firstinspires.ftc.teamcode.decode.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class LEDIndicator {

    public enum State {
        RED,
        GREEN,
        AMBER,
        OFF,
    }

    private final DigitalChannel redLED, greenLED;

    public LEDIndicator(HardwareMap hardwareMap, String greenName, String redName) {
        // Get the LED colors and touch sensor from the hardware map
        greenLED = hardwareMap.get(DigitalChannel.class, greenName);
        redLED = hardwareMap.get(DigitalChannel.class, redName);

        // change LED mode from input to output
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void setState(State ledColor) {
        greenLED.setState(ledColor == State.GREEN || ledColor == State.OFF);
        redLED.setState(ledColor == State.RED || ledColor == State.OFF);
    }
}