package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;
import org.firstinspires.ftc.teamcode.decode.util.prism.Color;
import org.firstinspires.ftc.teamcode.decode.util.prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.decode.util.prism.PrismAnimations;

public class LEDController {

    private static final int STRIP_LENGTH = 36;
    private static final int SMALL_STRIP_LENGTH = 6;
    private static final int BIG_STRIP_LENGTH = 12;
    private static final int BRIGHTNESS = 50;
    private static final int TARGET_FPS = 60;

    private final GoBildaPrismDriver prism;

    private final PrismAnimations.Solid llStrip = new PrismAnimations.Solid();
    private final PrismAnimations.Solid turretStrip = new PrismAnimations.Solid();

    private boolean initialized = false;

    private Pattern lastLLPattern = null;
    private Pattern lastTurretPattern = null;

    public static int LOOP_COUNTER = (1 << 5) - 1;

    public LEDController(HardwareMap hardwareMap) {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
    }

    public void update() {
        if ((LoopUtil.getLoops() & LOOP_COUNTER) == 0) {
            ensureInitialized();

            Pattern llPattern = computeLLPattern();
            Pattern turretPattern = computeTurretPattern();

            if (llPattern != lastLLPattern) {
                llStrip.setPrimaryColor(toColor(llPattern));
                prism.updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight.LAYER_0);
                lastLLPattern = llPattern;
            }

            if (turretPattern != lastTurretPattern) {
                turretStrip.setPrimaryColor(toColor(turretPattern));
                prism.updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight.LAYER_1);
                lastTurretPattern = turretPattern;
            }
        }
    }

    public void ensureInitialized() {
        if (initialized) return;

        prism.setStripLength(STRIP_LENGTH);
        prism.setTargetFPS(TARGET_FPS);
        prism.clearAllAnimations();

        // ---- RPM strip (LEDs 0–5) ----
        turretStrip.setBrightness(BRIGHTNESS);
        turretStrip.setStartIndex(2*BIG_STRIP_LENGTH);
        turretStrip.setStopIndex(2*BIG_STRIP_LENGTH+SMALL_STRIP_LENGTH - 1);
        turretStrip.setPrimaryColor(toColor(Pattern.OFF));

        // ---- Turret strip (LEDs 6–11) ----
        llStrip.setBrightness(BRIGHTNESS);
        llStrip.setStartIndex(2*BIG_STRIP_LENGTH+SMALL_STRIP_LENGTH);
        llStrip.setStopIndex(2*BIG_STRIP_LENGTH+(2 * SMALL_STRIP_LENGTH) - 1);
        llStrip.setPrimaryColor(toColor(Pattern.OFF));

        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, llStrip);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, turretStrip);

        lastLLPattern = Pattern.OFF;
        lastTurretPattern = Pattern.OFF;
        initialized = true;
    }

    // -------------------------
    // Pattern logic
    // -------------------------

    private Pattern computeLLPattern() {
        if (robot.limelight.getPoseEstimate(robot.drivetrain.getHeading()) == null) {
            return Pattern.RED;
        }
        return Pattern.GREEN;
    }

    private Pattern computeTurretPattern() {
        if (!robot.shooter.turret.isPIDInTolerance() || !robot.shooter.turret.isReadyToShoot()) {
            return Pattern.RED;
        }
        return Pattern.GREEN;
    }

    // -------------------------
    // Utilities
    // -------------------------

    public enum Pattern {
        GREEN,
        RED,
        ORANGE,
        OFF
    }

    private static Color toColor(Pattern p) {
        switch (p) {
            case GREEN:  return new Color(0, 255, 0);
            case RED:    return new Color(255, 0, 0);
            case ORANGE: return new Color(255, 165, 0);
            case OFF:
            default:     return new Color(0, 0, 0);
        }
    }
}