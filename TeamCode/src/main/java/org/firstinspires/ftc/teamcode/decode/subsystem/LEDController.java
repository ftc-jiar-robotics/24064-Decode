package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class LEDController {

    public enum Pattern {
        HOT_PINK,
        BLUE,
        WHITE,
        GREEN,
        YELLOW,
        ORANGE,
        RED,
        OFF
    }

    private static final int STRIP_LENGTH = 32; // set to your real LED count
    private static final int BRIGHTNESS = 100;

    private final GoBildaPrismDriver prism;

    private final PrismAnimations.Solid solid = new PrismAnimations.Solid();

    public LEDController(HardwareMap hardwareMap) {

        prism = hardwareMap.get(GoBildaPrismDriver.class, "blinkin");
        prism.setStripLength(STRIP_LENGTH);
    }

    /**
     * Update LEDs based on ball count and shooting outside triangle.
     * Shooting outside overrides ball count.
     * @param ballCount number of balls cued
     */
    public void update(int ballCount) {

        switch (ballCount) {
            case 3:
                setPattern(Pattern.HOT_PINK);
                break;
            case 2:
            case 1:
                setPattern(Pattern.BLUE);
                break;
            default:
                setPattern(Pattern.WHITE);
                break;
        }
    }

    public void showShooterTolerance() {
        switch (robot.shooter.get()) {
            case RUNNING:
                setPattern(Pattern.GREEN);
                break;
            case IDLE:
            case PREPPING:
            default:
                if (!robot.shooter.flywheel.isPIDInTolerance()) {
                    if (!robot.shooter.turret.isPIDInTolerance())
                        setPattern(Pattern.RED);
                    else
                        setPattern(Pattern.ORANGE);
                } else
                    setPattern(Pattern.YELLOW);
                break;
        }
    }

    public void setPattern(Pattern pattern) {
        Color c;
        switch (pattern) {
            case HOT_PINK: c = new Color(255, 105, 180); break;
            case BLUE:     c = new Color(0,   0,   255); break;
            case WHITE:    c = new Color(255, 255, 255); break;
            case GREEN:    c = new Color(0,   255, 0); break;
            case YELLOW:   c = new Color(255, 255, 0); break;
            case ORANGE:   c = new Color(255, 165, 0); break;
            case RED:      c = new Color(255, 0,   0); break;
            case OFF:
            default:       c = new Color(0,   0,   0); break;
        }

        solid.setPrimaryColor(c);
        solid.setStartIndex(0);
        solid.setStopIndex(STRIP_LENGTH - 1);
        solid.setBrightness(BRIGHTNESS);

        prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solid);
    }
}
