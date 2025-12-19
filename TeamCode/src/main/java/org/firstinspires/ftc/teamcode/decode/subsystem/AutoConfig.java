package org.firstinspires.ftc.teamcode.decode.subsystem;

import java.util.ArrayList;
import java.util.LinkedList;

public class AutoConfig {
    public enum Config {
        RED,
        BLUE,
        GOAL,
        AUDIENCE,
        PRELOAD,
        INTAKE_HP,
        INTAKE_FIRST,
        INTAKE_SECOND,
        INTAKE_THIRD,
        INTAKE_GATE,
        OPEN_GATE,
        SHOOT
    }

    private final ArrayList<Config> requirementList = new ArrayList<>();

    private int pointer = -1;

    public final String[] allianceChoices = {"RED", "BLUE"};
    public final String[] sideChoices = {null, null, "AUDIENCE", "GOAL"};
    public final String[] pathChoices = {null, null, null, null, "PRELOAD", "INTAKE HP", "INTAKE FIRST", "INTAKE SECOND", "INTAKE THIRD", "INTAKE GATE", "OPEN GATE", "SHOOT"};

    public void doConfig(String[] config, boolean moveCursorUp, boolean moveCursorDown, int screenNumber) {
        if (pointer == -1 || config[pointer] == null) while (config[++pointer] == null);

        if (requirementList.size() > screenNumber) pointer = requirementList.get(screenNumber).ordinal();

        if (moveCursorUp && pointer + 1 < config.length && config[pointer + 1] == null) {
            pointer++;
        }

        if (moveCursorDown && pointer - 1 >= 0 && config[pointer - 1] == null ) {
            pointer--;
        }

        for (int i = 0; i < config.length; i++) {
            if (config[i] != null) Common.dashTelemetry.addLine(i == pointer ? ">> " + config[i] : config[i]);
        }

        if (requirementList.size() > screenNumber) {
            requirementList.set(screenNumber, Config.values()[pointer]);
        } else requirementList.add(Config.values()[pointer]);
    }

    public void reduceRequireList(int screenNumber) {
        for (int i = requirementList.size() - 1; i >= screenNumber; i--) requirementList.remove(i);
    }
}
