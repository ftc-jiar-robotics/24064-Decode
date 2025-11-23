package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Artifact.PURPLE;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public enum Motif {

    GPP(GREEN, PURPLE, PURPLE),
    PGP(PURPLE, GREEN, PURPLE),
    PPG(PURPLE, PURPLE, GREEN);

    private final Artifact[] artifacts;

    private static final Motif[] motifs = values();

    Motif(Artifact... artifacts) {
        this.artifacts = artifacts;
    }

    /**
     * @return The artifact located at the provided index i in this motif
     */
    public Artifact get(int i) {
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


    /**
     * @param numArtifactsScored Number of artifacts in the classifier ramp
     * @return The motif pattern to score to satisfy the randomization
     */
    public Motif getEffectiveMotif(int numArtifactsScored) {
        return Motif.fromGreenIndex(this.ordinal() - numArtifactsScored);
    }

    /**
     * @param allowOneWrong Allow one wrong artifact color when scoring three artifacts
     * @param numArtifactsScored Number of artifacts in the classifier ramp
     * @param spindexerSlots Artifacts available in the spindexer
     * @return The order to score the artifacts in the spindexer
     */
    public ArrayList<Integer> getScoringOrder(boolean allowOneWrong, int numArtifactsScored, Artifact... spindexerSlots) {

        if (numArtifactsScored == 9)
            return new ArrayList<>();

        assert spindexerSlots.length == 3;

        Motif effectiveMotif = this.getEffectiveMotif(numArtifactsScored);

        allowOneWrong = allowOneWrong && EMPTY.numOccurrencesIn(spindexerSlots) == 0 && numArtifactsScored <= 6;

        int firstArtifactIndex = effectiveMotif.get(0).firstOccurrenceIn(spindexerSlots);
        int auditIndex = 1; // spindexer slot to check after first one

        // no artifact in spindexer matches first color of motif
        if (firstArtifactIndex == -1) {
            if (!allowOneWrong)
                return new ArrayList<>();

            // find occurrence of second motif color in spindexer
            firstArtifactIndex = effectiveMotif.get(1).firstOccurrenceIn(spindexerSlots) - 1;

            if (firstArtifactIndex == -2) // -2 because search result was followed by - 1
                return new ArrayList<>();

            auditIndex = 0;
        }

        ArrayList<Integer> scoringOrder = new ArrayList<>(Arrays.asList(0, 1, 2));

        Collections.rotate(scoringOrder, -firstArtifactIndex);

        boolean correctAudited = spindexerSlots[scoringOrder.get(auditIndex)] == effectiveMotif.get(auditIndex);

        if (!correctAudited && spindexerSlots[scoringOrder.get(2)] == effectiveMotif.get(auditIndex)) {
            Collections.swap(scoringOrder, auditIndex, 2);
            correctAudited = true;
        }

        boolean correctThird = spindexerSlots[scoringOrder.get(2)] == effectiveMotif.get(2);
        boolean scoringTwoThirds = correctThird && allowOneWrong;

        if ((!correctAudited || numArtifactsScored == 8) && !scoringTwoThirds)
            scoringOrder.subList(1, scoringOrder.size()).clear();
        else if (!correctThird || numArtifactsScored == 7)
            scoringOrder.remove(2);

        return scoringOrder;
    }

    /**
     * @param scoringOrder The order to score the artifacts in the spindexer
     * @param numArtifactsScored Number of artifacts in the classifier ramp
     * @param spindexerSlots Artifacts available in the spindexer
     * @return The number of points scored by following the provided scoringOrder
     */
    public int getScoreValue(ArrayList<Integer> scoringOrder, int numArtifactsScored, Artifact... spindexerSlots) {

        if (numArtifactsScored == 9)
            return 0;

        Motif effectiveMotif = getEffectiveMotif(numArtifactsScored);

        return scoringOrder.size() * 3 +
                            (!scoringOrder.isEmpty() && spindexerSlots[scoringOrder.get(0)] == effectiveMotif.artifacts[0] ? 2 : 0) +
                            (scoringOrder.size() > 1 && spindexerSlots[scoringOrder.get(1)] == effectiveMotif.artifacts[1] ? 2 : 0) +
                            (scoringOrder.size() > 2 && spindexerSlots[scoringOrder.get(2)] == effectiveMotif.artifacts[2] ? 2 : 0)
        ;
    }

    /**
     * @param allowOneWrong Allow one wrong artifact color when scoring three artifacts
     * @param numArtifactsScored Number of artifacts in the classifier ramp
     * @param spindexerSlots Artifacts available in the spindexer
     * @return The order to score the artifacts in the spindexer, as a {@link String}
     */
    public String getScoringInstructions(boolean allowOneWrong, int numArtifactsScored, Artifact... spindexerSlots) {
        ArrayList<Integer> scoringOrder = getScoringOrder(allowOneWrong, numArtifactsScored, spindexerSlots);
        if (scoringOrder.isEmpty())
            return "Continue intaking";
        StringBuilder s = new StringBuilder(String.format("Score slot%s ", scoringOrder.size() > 1 ? "s" : ""));
        for (int k : scoringOrder)
            s.append(k).append(" ");
        s.append("(");
        for (int i : scoringOrder)
            s.append(spindexerSlots[i].name().charAt(0));
        return s + ") for " + getScoreValue(scoringOrder, numArtifactsScored, spindexerSlots) + " pts";
    }

    public static void main(String[] args) {

        // edit these for testing:
        String classifierRamp = "PPG PGG GG_";
        boolean allowOneWrong = true;

        System.out.println("Classifier: " + classifierRamp);

        Artifact[] artifacts = Artifact.values();

        int numArtifactsScored = classifierRamp.replace(" ", "").replace("_", "").length();

        for (Motif randomization : Motif.motifs) {
            System.out.printf("Randomization: %s%nEffective: %s%nSpindexer: %n", randomization, randomization.getEffectiveMotif(numArtifactsScored));
            for (Artifact first : artifacts) for (Artifact second : artifacts) for (Artifact third : artifacts) {
                System.out.printf("     %s%s%s --> %s%n",
                        first.name().replace("E", "_").charAt(0),
                        second.name().replace("E", "_").charAt(0),
                        third.name().replace("E", "_").charAt(0),
                        randomization.getScoringInstructions(allowOneWrong, numArtifactsScored, first, second, third)
                );
            }
        }
    }
}
