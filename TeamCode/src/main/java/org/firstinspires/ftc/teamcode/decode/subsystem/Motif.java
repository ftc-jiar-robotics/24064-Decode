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

        return Motif.fromGreenIndex(ordinal() - i);
    }

    /**
     * @param spindexerSlots Artifacts available in the spindexer (0 = front, 1 = back left, 2 = back right)
     * @return The order to score the artifacts in the spindexer
     */
    public int[] getScoringOrder(Artifact... spindexerSlots) {

        int firstArtifactIndex = first.firstOccurrenceIn(spindexerSlots);

        if (firstArtifactIndex == -1)
            return new int[]{};

        int 
                length = spindexerSlots.length,
                secondArtifactIndex = (firstArtifactIndex + 1) % length,
                thirdArtifactIndex = (firstArtifactIndex + 2) % length;

        if (spindexerSlots[secondArtifactIndex] != second && spindexerSlots[thirdArtifactIndex] == second) {
            int temp = secondArtifactIndex;
            secondArtifactIndex = thirdArtifactIndex;
            thirdArtifactIndex = temp;
        }

        return 
                spindexerSlots[secondArtifactIndex] != second ?
                        new int[]{firstArtifactIndex} :
                        spindexerSlots[thirdArtifactIndex] != third ?
                                new int[]{firstArtifactIndex, secondArtifactIndex} :
                                new int[]{firstArtifactIndex, secondArtifactIndex, thirdArtifactIndex}
        ;
    }

    public String getScoringInstructions(Artifact... spindexerSlots) {
        int[] scoringOrder = getScoringOrder(spindexerSlots);
        if (scoringOrder.length == 0)
            return "Continue intaking";
        StringBuilder s = new StringBuilder(String.format("Score slot%s ", scoringOrder.length > 1 ? "s" : ""));
        for (int k : scoringOrder)
            s.append(k).append(" ");
        s.append("(");
        for (int i : scoringOrder)
            s.append(spindexerSlots[i].name().charAt(0));
        return s.toString() + ')';
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
                        System.out.println("Motif: " + motif + ", Spindexer: " + (c1 == 'E' ? '_' : c1) + (c2 == 'E' ? '_' : c2) + (c3 == 'E' ? '_' : c3) + " --> " +
                                motif.getScoringInstructions(first, second, third));
                    }
    }
}
