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
    private final PrismAnimations.Solid intakeFrontRightStrip = new PrismAnimations.Solid();
    private final PrismAnimations.Solid intakeFrontLeftStrip = new PrismAnimations.Solid();
    private final PrismAnimations.Solid intakeBackRightStrip = new PrismAnimations.Solid();
    private final PrismAnimations.Solid intakeBackLeftStrip = new PrismAnimations.Solid();
    private final PrismAnimations.Solid feederRightStrip = new PrismAnimations.Solid();
    private final PrismAnimations.Solid feederLeftStrip = new PrismAnimations.Solid();




    private boolean initialized = false;

    private Pattern lastLLPattern = null;
    private Pattern lastTurretPattern = null;
    private Pattern lastFeederPattern = null;
    private Pattern lastIntakeFrontPattern = null;
    private Pattern lastIntakeBackPattern = null;

    public static int LOOP_COUNTER = (1 << 3) - 1;

    public LEDController(HardwareMap hardwareMap) {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
    }

    public void update() {
        if ((LoopUtil.getLoops() & LOOP_COUNTER) == 0) {
            ensureInitialized();

            Pattern llPattern = computeLLPattern();
            Pattern turretPattern = computeTurretPattern();

            Pattern intakeFrontPattern = computeIntakeFrontPattern();
            Pattern intakeBackPattern = computeIntakeBackPattern();
            Pattern feederPattern = computeFeederPattern();

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

            if (intakeFrontPattern != lastIntakeFrontPattern) {
                intakeFrontRightStrip.setPrimaryColor(toColor(intakeFrontPattern));
                intakeFrontLeftStrip.setPrimaryColor(toColor(intakeFrontPattern));
                prism.updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight.LAYER_2);
                prism.updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight.LAYER_3);
                lastIntakeFrontPattern = intakeFrontPattern;
            }

            if (intakeBackPattern != lastIntakeBackPattern) {
                intakeBackLeftStrip.setPrimaryColor(toColor(intakeBackPattern));
                intakeBackRightStrip.setPrimaryColor(toColor(intakeBackPattern));
                prism.updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight.LAYER_4);
                prism.updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight.LAYER_5);
                lastIntakeBackPattern = intakeBackPattern;
            }

            if (feederPattern != lastFeederPattern) {
                feederRightStrip.setPrimaryColor(toColor(feederPattern));
                feederLeftStrip.setPrimaryColor(toColor(feederPattern));
                prism.updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight.LAYER_6);
                prism.updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight.LAYER_7);
                lastFeederPattern = feederPattern;
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

        // ---- intakeFrontRight Strip ----
        intakeFrontRightStrip.setBrightness(BRIGHTNESS);
        intakeFrontRightStrip.setStartIndex(20);
        intakeFrontRightStrip.setStopIndex(23);
        intakeFrontRightStrip.setPrimaryColor(toColor(Pattern.OFF));


        // ---- intakeFrontLeft Strip ----
        intakeFrontLeftStrip.setBrightness(BRIGHTNESS);
        intakeFrontLeftStrip.setStartIndex(0);
        intakeFrontLeftStrip.setStopIndex(3);
        intakeFrontLeftStrip.setPrimaryColor(toColor(Pattern.OFF));

        // ---- intakeBackRight Strip ----
        intakeBackRightStrip.setBrightness(BRIGHTNESS);
        intakeBackRightStrip.setStartIndex(16);
        intakeBackRightStrip.setStopIndex(19);
        intakeBackRightStrip.setPrimaryColor(toColor(Pattern.OFF));

        // ---- intakeBackLeft Strip ----
        intakeBackLeftStrip.setBrightness(BRIGHTNESS);
        intakeBackLeftStrip.setStartIndex(4);
        intakeBackLeftStrip.setStopIndex(7);
        intakeBackLeftStrip.setPrimaryColor(toColor(Pattern.OFF));

        // ---- feederRight Strip ----
        feederRightStrip.setBrightness(BRIGHTNESS);
        feederRightStrip.setStartIndex(12);
        feederRightStrip.setStopIndex(15);
        feederRightStrip.setPrimaryColor(toColor(Pattern.OFF));

        // ---- feederLeft Strip ----
        feederLeftStrip.setBrightness(BRIGHTNESS);
        feederLeftStrip.setStartIndex(8);
        feederLeftStrip.setStopIndex(11);
        feederLeftStrip.setPrimaryColor(toColor(Pattern.OFF));




        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, llStrip);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, turretStrip);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_2, intakeFrontRightStrip);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_3, intakeFrontLeftStrip);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_4, intakeBackRightStrip);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_5, intakeBackLeftStrip);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_6, feederRightStrip);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_7, feederLeftStrip);

        lastLLPattern = Pattern.OFF;
        lastTurretPattern = Pattern.OFF;
        lastFeederPattern = Pattern.OFF ;
        lastIntakeFrontPattern = Pattern.OFF;
        lastIntakeBackPattern = Pattern.OFF;
        initialized = true;
    }

    // -------------------------
    // Pattern logic
    // -------------------------

    private Pattern computeLLPattern() {
        if (robot.autoAim.getTurretPosePedro() == null) {
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

    private Pattern computeIntakeBackPattern() {
        if (!robot.shooter.isBallInIntakeBack()) {
            return Pattern.RED;
        }
        return Pattern.GREEN;
    }

    private Pattern computeIntakeFrontPattern() {
        if (!robot.shooter.isBallInIntakeFront()) {
            return Pattern.RED;
        }
        return Pattern.GREEN;
    }
    private Pattern computeFeederPattern() {
        if (!robot.shooter.isBallInFeeder()) {
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