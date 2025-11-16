package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

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
     */
    public void update(int ballCount) {

        switch (ballCount) {
            case 3:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                break;
            case 2:
            case 1:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                break;
            default:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                break;
        }

    }

    public void showShooterTolerance() {
        switch (robot.shooter.get()) {
            case RUNNING:
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            case IDLE:
            case PREPPING:
            default:
                if (!robot.shooter.flywheel.isPIDInTolerance()) {
                    if (!robot.shooter.turret.isPIDInTolerance())
                        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    else
                        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                } else
                    blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;
        }
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern color) {
        blinkin.setPattern(color);
    }
}
