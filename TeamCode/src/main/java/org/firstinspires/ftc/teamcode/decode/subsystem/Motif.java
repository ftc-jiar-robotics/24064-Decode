package org.firstinspires.ftc.teamcode.decode.subsystem;

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
     * APRIL TAG IDs (21 22 23) CAN BE PASSED DIRECTLY TO THIS
     * @param greenIndex    Index of the green artifact in the motif GPP = 0, PGP = 1, PPG = 2
     * @return              The {@link Motif} that has a green artifact in the specified position
     */
    public static Motif fromGreenIndex(int greenIndex) {
        int length = motifs.length;
        return motifs[(greenIndex % length + length) % length];
    }

    /**
     * @param classifierRamp    The artifacts currently in the classifier ramp (starting from the gate)
     * @return                  The motif pattern to score, {@link null} if classifier is full
     */
    public Motif getEffectiveMotif(Artifact... classifierRamp) {

        int i = Artifact.EMPTY.firstOccurrenceIn(classifierRamp);
        if (i == -1) return null;

        return Motif.fromGreenIndex((ordinal() - i) % motifs.length);
    }

    /**
     * @param firstArtifactIndex Spindexer slot index of the first artifact to score
     * @param spindexerSlots Artifacts available in the spindexer (0 = front, 1 = back left, 2 = back right)
     * @return Run spindexer counter-clockwise (CCW) when scoring motif
     */
    public boolean scoreCounterClockwise(int firstArtifactIndex, Artifact... spindexerSlots) {
        int length = spindexerSlots.length;
        Artifact secondArtifact = spindexerSlots[(firstArtifactIndex + 1) % length];
        Artifact thirdArtifact = spindexerSlots[(firstArtifactIndex + 2) % length];

        // if the next artifact (clockwise from the first one) is NOT the  next motif color,
        // and the third IS the next motif color, run the spindexer the other way
        return secondArtifact != second && thirdArtifact == second;
    }

    /**
     * @param scoringCounterClockwise Running spindexer counter-clockwise (CCW) when scoring motif
     * @param firstArtifactIndex Spindexer slot index of the first artifact to score
     * @param spindexerSlots Artifacts available in the spindexer (0 = front, 1 = back left, 2 = back right)
     * @return How many more artifacts to score after the first one (Can be 0, 1, or 2)
     */
    public int numAdditionalArtifacts(boolean scoringCounterClockwise, int firstArtifactIndex, Artifact... spindexerSlots) {
        int length = spindexerSlots.length;
        Artifact secondArtifact = spindexerSlots[(firstArtifactIndex + 1) % length];
        Artifact thirdArtifact = spindexerSlots[(firstArtifactIndex + 2) % length];
        if (scoringCounterClockwise)
            thirdArtifact = secondArtifact;

        return scoringCounterClockwise || secondArtifact == second ? thirdArtifact == third ? 2 : 1 : 0;
    }

    public String scoringInstructions(int firstArtifactIndex, Artifact... spindexerSlots) {
        if (firstArtifactIndex == -1) return "Continue intaking";
        boolean counterClockwise = scoreCounterClockwise(firstArtifactIndex, spindexerSlots);
        int additionalArtifacts = numAdditionalArtifacts(counterClockwise, firstArtifactIndex, spindexerSlots);
        return "Score slot " + firstArtifactIndex + " artifact" + (additionalArtifacts == 0 ? "" : " first, then score " + additionalArtifacts + " more artifact" + (additionalArtifacts == 1 ? "" : "s") + " by rotating " + (counterClockwise ? "CCW" : "CW"));
    }

    public static void main(String[] args) {
        Artifact[] artifacts = Artifact.values();

        for (Motif motif : Motif.motifs)
            for (Artifact first : artifacts)
                for (Artifact second : artifacts)
                    for (Artifact third : artifacts) {
                        char
                                c1 = first.name().charAt(0),
                                c2 = second.name().charAt(0),
                                c3 = third.name().charAt(0);
                        int firstArtifactIndex = motif.first.firstOccurrenceIn(first, second, third);
                        System.out.println("Motif: " + motif + ", Spindexer: " + (c1 == 'E' ? '_' : c1) + (c2 == 'E' ? '_' : c2) + (c3 == 'E' ? '_' : c3) + " --> " +
                                motif.scoringInstructions(firstArtifactIndex, first, second, third));
                    }
    }
}
