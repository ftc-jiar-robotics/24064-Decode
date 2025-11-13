package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Artifact.PURPLE;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public enum Motif {

    GPP(GREEN, PURPLE, PURPLE),
    PGP(PURPLE, GREEN, PURPLE),
    PPG(PURPLE, PURPLE, GREEN);

    private final Artifact[] artifacts;

    private static final Motif[] motifs = values();

    Motif(Artifact... artifacts) {
        this.artifacts = artifacts;
    }

    public Artifact getArtifact(int i) {
        return artifacts[wrap(i, 0, artifacts.length)];
    }

    /**
     * @param num   Number to wrap
     * @param from  Lower bound (INCLUSIVE)
     * @param to    Upper bound (EXCLUSIVE), must be greater than from
     * @return      The number wrapped to the range [from, to)
     */
    public static int wrap(int num, int from, int to) {
        int mod = to - from;
        return ((num - from) % mod + mod) % mod + from;
    }

    /**
     * APRIL TAG IDs (21 22 23) CAN BE PASSED DIRECTLY TO THIS
     * @param greenIndex    Index of the green artifact in the motif GPP = 0, PGP = 1, PPG = 2
     * @return              The {@link Motif} that has a green artifact in the specified position
     */
    public static Motif fromGreenIndex(int greenIndex) {
        return motifs[wrap(greenIndex, 0, motifs.length)];
    }

    public Motif getEffectiveMotif(int numClassifierSlotsEmpty) {
        return Motif.fromGreenIndex(ordinal() - (9 - numClassifierSlotsEmpty));
    }

    /**
     * @param numClassifierSlotsEmpty Number of empty spots in the classifier ramp
     * @param spindexerSlots Artifacts available in the spindexer (0 = front, 1 = back left, 2 = back right)
     * @param allowOneWrong Allow one wrong artifact color when scoring three artifacts
     * @return The order to score the artifacts in the spindexer
     */
    public int[] getScoringOrder(boolean allowOneWrong, int numClassifierSlotsEmpty, Artifact... spindexerSlots) {

        ArrayList<Integer> scoringOrder = new ArrayList<>(Arrays.asList(0, 1, 2));

        assert spindexerSlots.length == scoringOrder.size();

        if (numClassifierSlotsEmpty == 0)
            return new int[]{};

        Motif effectiveMotif = getEffectiveMotif(numClassifierSlotsEmpty);

        allowOneWrong = allowOneWrong && EMPTY.countOccurrencesIn(spindexerSlots) == 0 && numClassifierSlotsEmpty >= 3;

        int firstArtifactIndex = effectiveMotif.getArtifact(0).firstOccurrenceIn(spindexerSlots);
        int auditIndex;

        if (firstArtifactIndex == -1) {
            if (!allowOneWrong)
                return new int[]{};

            int secondArtifactIndex = effectiveMotif.getArtifact(1).firstOccurrenceIn(spindexerSlots);

            if (secondArtifactIndex == -1)
                return new int[]{};

            firstArtifactIndex = secondArtifactIndex - 1;
            auditIndex = 0;
        } else auditIndex = 1;

        Collections.rotate(scoringOrder, -firstArtifactIndex);

        boolean correctAudited = spindexerSlots[scoringOrder.get(auditIndex)] == effectiveMotif.getArtifact(auditIndex);

        if (!correctAudited && spindexerSlots[scoringOrder.get(2)] == effectiveMotif.getArtifact(auditIndex)) {
            Collections.swap(scoringOrder, auditIndex, 2);
            correctAudited = true;
        }

        boolean correctThird = spindexerSlots[scoringOrder.get(2)] == effectiveMotif.getArtifact(2);
        boolean scoringTwoThirds = correctThird && allowOneWrong;

        if ((!correctAudited || numClassifierSlotsEmpty < 2) && !scoringTwoThirds)
            scoringOrder.subList(1, scoringOrder.size()).clear();
        else if (!correctThird || numClassifierSlotsEmpty < 3)
            scoringOrder.remove(2);

        return toIntArray(scoringOrder);
    }

    private static int[] toIntArray(List<Integer> list)  {
        int[] ret = new int[list.size()];
        int i = 0;
        for (Integer e : list)
            ret[i++] = e;
        return ret;
    }

    public int getScoreValue(int[] scoringOrder, int numClassifierSlotsEmpty, Artifact... spindexerSlots) {

        if (numClassifierSlotsEmpty == 0)
            return 0;

        Motif effectiveMotif = getEffectiveMotif(numClassifierSlotsEmpty);

        return scoringOrder.length * 3 +
                            (scoringOrder.length > 0 && spindexerSlots[scoringOrder[0]] == effectiveMotif.artifacts[0] ? 2 : 0) +
                            (scoringOrder.length > 1 && spindexerSlots[scoringOrder[1]] == effectiveMotif.artifacts[1] ? 2 : 0) +
                            (scoringOrder.length > 2 && spindexerSlots[scoringOrder[2]] == effectiveMotif.artifacts[2] ? 2 : 0)
        ;
    }

    public String getScoringInstructions(boolean allowOneWrong, int numClassifierSlotsEmpty, Artifact... spindexerSlots) {
        int[] scoringOrder = getScoringOrder(allowOneWrong, numClassifierSlotsEmpty, spindexerSlots);
        if (scoringOrder.length == 0)
            return "Continue intaking";
        StringBuilder s = new StringBuilder(String.format("Score slot%s ", scoringOrder.length > 1 ? "s" : ""));
        for (int k : scoringOrder)
            s.append(k).append(" ");
        s.append("(");
        for (int i : scoringOrder)
            s.append(spindexerSlots[i].name().charAt(0));
        return s + ") for " + getScoreValue(scoringOrder, numClassifierSlotsEmpty, spindexerSlots) + " pts";
    }

    public static void main(String[] args) {

        // edit these for testing:
        String classifierRamp = "PPG PGP ___";
        boolean allowOneWrong = true;

        System.out.println("Classifier: " + classifierRamp);

        Artifact[] artifacts = Artifact.values();

        int numClassifierSlotsEmpty = classifierRamp.replace(" ", "").length() - classifierRamp.replace(" ", "").replace("_", "").length();

        for (Motif randomization : Motif.motifs)
            for (Artifact first : artifacts)
                for (Artifact second : artifacts)
                    for (Artifact third : artifacts) {
                        System.out.printf("Randomization: %s, Spindexer: %s%s%s --> %s%n",
                                randomization,
                                first.name().replace("E", "_").charAt(0),
                                second.name().replace("E", "_").charAt(0),
                                third.name().replace("E", "_").charAt(0),
                                randomization.getScoringInstructions(allowOneWrong, numClassifierSlotsEmpty, first, second, third)
                        );
                    }
    }
}
