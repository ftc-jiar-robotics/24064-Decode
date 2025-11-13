package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Artifact.PURPLE;

public enum Motif {

    GPP(GREEN, PURPLE, PURPLE),
    PGP(PURPLE, GREEN, PURPLE),
    PPG(PURPLE, PURPLE, GREEN);

    public final Artifact first, second, third;

    private static final Motif[] motifs = values();

    Motif(Artifact first, Artifact second, Artifact third) {
        this.first = first;
        this.second = second;
        this.third = third;
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

        if (numClassifierSlotsEmpty == 0)
            return new int[]{};

        Motif effectiveMotif = getEffectiveMotif(numClassifierSlotsEmpty);

        int length = spindexerSlots.length;
        assert length == 3;

        allowOneWrong = allowOneWrong && EMPTY.countOccurrencesIn(spindexerSlots) == 0 && numClassifierSlotsEmpty >= 3;

        int firstArtifactIndex = effectiveMotif.first.firstOccurrenceIn(spindexerSlots);

        if (firstArtifactIndex == -1) {
            if (!allowOneWrong)
                return new int[]{};

            int secondArtifactIndex = effectiveMotif.second.firstOccurrenceIn(spindexerSlots);

            if (secondArtifactIndex == -1)
                return new int[]{};

            firstArtifactIndex = wrap(secondArtifactIndex - 1, 0, length);
            int thirdArtifactIndex = wrap(secondArtifactIndex + 1, 0, length);

            return
                    spindexerSlots[thirdArtifactIndex] == effectiveMotif.third ?
                            new int[]{firstArtifactIndex, secondArtifactIndex, thirdArtifactIndex} :
                            spindexerSlots[firstArtifactIndex] == effectiveMotif.third ?
                                    new int[]{thirdArtifactIndex, secondArtifactIndex, firstArtifactIndex} :
                                    new int[]{}
            ;
        }

        int secondArtifactIndex = wrap(firstArtifactIndex + 1, 0, length);
        int thirdArtifactIndex = wrap(firstArtifactIndex + 2, 0, length);

        if (spindexerSlots[secondArtifactIndex] != effectiveMotif.second && spindexerSlots[thirdArtifactIndex] == effectiveMotif.second) {
            int temp = secondArtifactIndex;
            secondArtifactIndex = thirdArtifactIndex;
            thirdArtifactIndex = temp;
        }

        boolean validSecond = numClassifierSlotsEmpty >= 2 && spindexerSlots[secondArtifactIndex] == effectiveMotif.second;
        boolean validThird = numClassifierSlotsEmpty >= 3 && spindexerSlots[thirdArtifactIndex] == effectiveMotif.third;
        return
                validSecond ?
                        validThird ?
                                new int[]{firstArtifactIndex, secondArtifactIndex, thirdArtifactIndex} :
                                new int[]{firstArtifactIndex, secondArtifactIndex} :
                        allowOneWrong && validThird ?
                                new int[]{firstArtifactIndex, secondArtifactIndex, thirdArtifactIndex} :
                                new int[]{firstArtifactIndex}
        ;
    }

    public int getScoreValue(int[] scoringOrder, int numClassifierSlotsEmpty, Artifact... spindexerSlots) {

        if (numClassifierSlotsEmpty == 0)
            return 0;

        Motif effectiveMotif = getEffectiveMotif(numClassifierSlotsEmpty);

        return scoringOrder.length * 3 +
                            (scoringOrder.length > 0 && spindexerSlots[scoringOrder[0]] == effectiveMotif.first ? 2 : 0) +
                            (scoringOrder.length > 1 && spindexerSlots[scoringOrder[1]] == effectiveMotif.second ? 2 : 0) +
                            (scoringOrder.length > 2 && spindexerSlots[scoringOrder[2]] == effectiveMotif.third ? 2 : 0)
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
        String classifierRamp = "PPGPGP___";
        boolean allowOneWrong = true;

        System.out.println("Classifier: " + classifierRamp);

        Artifact[] artifacts = Artifact.values();

        int numClassifierSlotsEmpty = classifierRamp.length() - classifierRamp.replace("_", "").length();

        for (Motif motif : Motif.motifs)
            for (Artifact first : artifacts)
                for (Artifact second : artifacts)
                    for (Artifact third : artifacts) {
                        char
                                c1 = first.name().charAt(0),
                                c2 = second.name().charAt(0),
                                c3 = third.name().charAt(0);
                        System.out.println("Randomization: " + motif + ", Spindexer: " + (c1 == 'E' ? '_' : c1) + (c2 == 'E' ? '_' : c2) + (c3 == 'E' ? '_' : c3) + " --> " +
                                motif.getScoringInstructions(allowOneWrong, numClassifierSlotsEmpty, first, second, third));
                    }
    }
}
