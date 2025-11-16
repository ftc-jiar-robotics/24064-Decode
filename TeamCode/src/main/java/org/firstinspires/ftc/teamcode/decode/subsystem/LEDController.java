package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDController {

    private final RevBlinkinLedDriver blinkin;

    public LEDController(HardwareMap hardwareMap) {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }

    /**
     * Update LEDs based on ball count and shooting outside triangle.
     * Shooting outside overrides ball count.
     * @param ballCount number of balls cued
     * @param shootingInside true if trying to shoot outside the triangle
     */
    public void update(int ballCount, boolean shootingInside) {
        if (!shootingInside) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
        } else {
            if (ballCount > 3) {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
            } else {
                switch (ballCount) {
                    case 3:
                        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                        break;
                    case 2:
                        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE);
                        break;
                    case 1:
                        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_FOREST_PALETTE);
                        break;
                    default:
                        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY);
                        break;
                }
            }
        }
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern color) {
        blinkin.setPattern(color);
    }
}
