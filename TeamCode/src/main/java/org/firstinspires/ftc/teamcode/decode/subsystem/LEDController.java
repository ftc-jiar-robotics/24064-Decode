package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class LEDController {

    // ---- Prism config ----
    private static final int STRIP_LENGTH = 32;
    private static final int BRIGHTNESS = 50;

    /**
     * "Improvements" version:
     * Layer 0 shows ballCount across the whole strip.
     * Layer 1 shows shooter tolerance on a small segment so it doesn't hide ballCount.
     *
     * If you want shooter to override the whole strip, set this to STRIP_LENGTH.
     */
    private static final int SHOOTER_SEGMENT_LENGTH = 4;

    private final GoBildaPrismDriver prism;

    private final PrismAnimations.Solid ballSolid = new PrismAnimations.Solid();
    private final PrismAnimations.Solid shooterSolid = new PrismAnimations.Solid();

    private boolean inserted = false;
    private Pattern lastBallPattern = null;
    private Pattern lastShooterPattern = null;

    public LEDController(HardwareMap hardwareMap) {
        // NOTE: update your Robot Config name to "prism" (instead of "blinkin")
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
    }

    /**
     * Update LEDs based on ball count (layer 0) and shooter tolerance (layer 1).
     * Uses caching so we only send I2C updates when the pattern actually changes.
     *
     * @param ballCount number of balls queued
     */
    public void update(int ballCount) {
        ensureInserted();

        // Layer 0: ball count (same mapping as REV code)
        Pattern ballPattern;
        switch (ballCount) {
            case 3:
                ballPattern = Pattern.HOT_PINK;
                break;
            case 2:
            case 1:
                ballPattern = Pattern.BLUE;
                break;
            default:
                ballPattern = Pattern.WHITE;
                break;
        }

        // Layer 1: shooter tolerance (same mapping as REV code)
        Pattern shooterPattern = computeShooterPattern();

        // Only push updates when something changed
        if (ballPattern != lastBallPattern) {
            ballSolid.setPrimaryColor(toPrismColor(ballPattern));
            prism.updateAnimationFromIndex(LayerHeight.LAYER_0);
            lastBallPattern = ballPattern;
        }

        if (shooterPattern != lastShooterPattern) {
            shooterSolid.setPrimaryColor(toPrismColor(shooterPattern));
            prism.updateAnimationFromIndex(LayerHeight.LAYER_1);
            lastShooterPattern = shooterPattern;
        }
    }

    /**
     * Kept for compatibility with old call sites.
     * This updates ONLY the shooter segment (layer 1).
     */
    public void showShooterTolerance() {
        ensureInserted();
        Pattern shooterPattern = computeShooterPattern();

        if (shooterPattern != lastShooterPattern) {
            shooterSolid.setPrimaryColor(toPrismColor(shooterPattern));
            prism.updateAnimationFromIndex(LayerHeight.LAYER_1);
            lastShooterPattern = shooterPattern;
        }
    }

    /**
     * Manual override helper (similar  to old setPattern).
     * This sets the whole strip ball layer to the pattern.
     */
    public void setPattern(Pattern pattern) {
        ensureInserted();
        ballSolid.setPrimaryColor(toPrismColor(pattern));
        prism.updateAnimationFromIndex(LayerHeight.LAYER_0);
        lastBallPattern = pattern;
    }


    private void ensureInserted() {
        if (inserted) return;

        prism.setStripLength(STRIP_LENGTH);

        // Layer 0 (ball count): full strip
        ballSolid.setBrightness(BRIGHTNESS);
        ballSolid.setStartIndex(0);
        ballSolid.setStopIndex(STRIP_LENGTH - 1);
        ballSolid.setPrimaryColor(toPrismColor(Pattern.WHITE));

        // Layer 1 (shooter tolerance): small segment by default (doesn't hide layer 0)
        shooterSolid.setBrightness(BRIGHTNESS);
        shooterSolid.setStartIndex(0);
        shooterSolid.setStopIndex(Math.max(0, Math.min(STRIP_LENGTH - 1, SHOOTER_SEGMENT_LENGTH - 1)));
        shooterSolid.setPrimaryColor(toPrismColor(Pattern.OFF));

        // Insert both once; after that we only call updateAnimationFromIndex(...)
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, ballSolid);
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_1, shooterSolid);

        inserted = true;
        lastBallPattern = Pattern.WHITE;
        lastShooterPattern = Pattern.OFF;
    }

    private Pattern computeShooterPattern() {
        switch (robot.shooter.get()) {
            case RUNNING:
                return Pattern.GREEN;

            case IDLE:
            case PREPPING:
            default:
                if (!robot.shooter.flywheel.isPIDInTolerance()) {
                    if (!robot.shooter.turret.isPIDInTolerance()) {
                        return Pattern.RED;
                    } else {
                        return Pattern.ORANGE;
                    }
                } else {
                    return Pattern.YELLOW;
                }
        }
    }

    public enum Pattern {
        HOT_PINK,
        BLUE,
        WHITE,
        GREEN,
        YELLOW,
        ORANGE,
        RED,
        OFF,
    }

    private static Color toPrismColor(Pattern pattern) {
        switch (pattern) {
            case HOT_PINK:
                return new Color(255, 105, 180);
            case BLUE:
                return new Color(0, 0, 255);
            case WHITE:
                return new Color(255, 255, 255);
            case GREEN:
                return new Color(0, 255, 0);
            case YELLOW:
                return new Color(255, 255, 0);
            case ORANGE:
                return new Color(255, 165, 0);
            case RED:
                return new Color(255, 0, 0);
            case OFF:
            default:
                return new Color(0, 0, 0);
        }
    }
}
