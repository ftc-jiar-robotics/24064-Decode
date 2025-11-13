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

    /**
     * @param classifierRamp The artifacts currently in the classifier ramp (starting from the gate)
     * @param spindexerSlots Artifacts available in the spindexer (0 = front, 1 = back left, 2 = back right)
     * @param allowOneWrong Allow one wrong artifact color when scoring three artifacts
     * @return The order to score the artifacts in the spindexer
     */
    public int[] getScoringOrder(boolean allowOneWrong, Artifact[] classifierRamp, Artifact... spindexerSlots) {

        assert classifierRamp.length == 9;

        int numEmptySlots = EMPTY.countOccurrencesIn(classifierRamp);
        if (numEmptySlots == 0)
            return new int[]{};

        Motif effectiveMotif = Motif.fromGreenIndex(ordinal() - EMPTY.firstOccurrenceIn(classifierRamp));

        int length = spindexerSlots.length;
        assert length == 3;

        allowOneWrong = allowOneWrong && EMPTY.countOccurrencesIn(spindexerSlots) == 0;

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

        boolean validSecond = numEmptySlots >= 2 && spindexerSlots[secondArtifactIndex] == effectiveMotif.second;
        boolean validThird = numEmptySlots >= 3 && spindexerSlots[thirdArtifactIndex] == effectiveMotif.third;
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

    public int getScoreValue(int[] scoringOrder, Artifact[] classifierRamp, Artifact... spindexerSlots) {

        assert classifierRamp.length == 9;

        if (EMPTY.countOccurrencesIn(classifierRamp) == 0)
            return 0;

        Motif effectiveMotif = Motif.fromGreenIndex(ordinal() - EMPTY.firstOccurrenceIn(classifierRamp));

        return scoringOrder.length * 3 +
                            (scoringOrder.length > 0 && spindexerSlots[scoringOrder[0]] == first ? 2 : 0) +
                            (scoringOrder.length > 1 && spindexerSlots[scoringOrder[1]] == first ? 2 : 0) +
                            (scoringOrder.length > 2 && spindexerSlots[scoringOrder[2]] == first ? 2 : 0)
        ;
    }

    public String getScoringInstructions(boolean allowOneWrong, Artifact[] classifierRamp, Artifact... spindexerSlots) {
        int[] scoringOrder = getScoringOrder(allowOneWrong, classifierRamp, spindexerSlots);
        if (scoringOrder.length == 0)
            return "Continue intaking";
        StringBuilder s = new StringBuilder(String.format("Score slot%s ", scoringOrder.length > 1 ? "s" : ""));
        for (int k : scoringOrder)
            s.append(k).append(" ");
        s.append("(");
        for (int i : scoringOrder)
            s.append(spindexerSlots[i].name().charAt(0));
        return s.toString() + ") for " + getScoreValue(scoringOrder, classifierRamp, spindexerSlots) + " pts";
    }

    public static void main(String[] args) {

        Artifact[] classifierRamp = {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY};
        System.out.print("Classifier:");
        for (Artifact a : classifierRamp) {
            System.out.print(" " + a);
        }
        System.out.println();

        Artifact[] artifacts = Artifact.values();

        for (Motif motif : Motif.motifs)
            for (Artifact first : artifacts)
                for (Artifact second : artifacts)
                    for (Artifact third : artifacts) {
                        char
                                c1 = first.name().charAt(0),
                                c2 = second.name().charAt(0),
                                c3 = third.name().charAt(0);
                        System.out.println("Randomization: " + motif + ", Spindexer: " + (c1 == 'E' ? '_' : c1) + (c2 == 'E' ? '_' : c2) + (c3 == 'E' ? '_' : c3) + " --> " +
                                motif.getScoringInstructions(true, classifierRamp, first, second, third));
                    }
    }
}
