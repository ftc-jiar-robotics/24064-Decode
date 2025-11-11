package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Motifs.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Motifs.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Motifs.Artifact.PURPLE;
import static java.lang.Math.PI;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.HSV;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@Configurable
public class Motifs {

    public static HSV
            minPurple = new HSV(
                    205,
                    0.55,
                    0.01
            ),
            maxPurple = new HSV(
                    225,
                    1,
                    0.35
            ),
            minGreen = new HSV(
                    130,
                    0.5,
                    0.01
            ),
            maxGreen = new HSV(
                    160,
                    1,
                    0.2
            );

    public enum Artifact {
        PURPLE,
        GREEN,
        EMPTY;

        public static Artifact fromHSV(HSV hsv) {
            return
                    hsv.between(minPurple, maxPurple) ? PURPLE :
                    hsv.between(minGreen, maxGreen) ? GREEN :
                    EMPTY;
        }
    }

    public enum Motif {
        GPP(PI / 6,     GREEN, PURPLE, PURPLE),
        PGP(PI,         PURPLE, GREEN, PURPLE),
        PPG(-PI / 6,    PURPLE, PURPLE, GREEN);

        /**
         * Where to move our green artifact (the one inside the spindexer) to
         */
        private final double greenArtifactRadians;

        private final Artifact[] artifacts;

        private static final Motif[] motifs = values();

        Motif(double greenArtifactRadians, Artifact... artifacts) {
            this.greenArtifactRadians = greenArtifactRadians;
            this.artifacts = artifacts;
        }

        public Artifact[] toArray() {
            return artifacts.clone();
        }

        public int getGreenIndex() {
            return ordinal();
        }

        public static Motif fromArray(Artifact... artifacts) {
            assert artifacts.length == 3;
            return fromGreenIndex(indexOf(GREEN, artifacts));
        }

        public static Motif fromGreenIndex(int greenIndex) {
            return motifs[(greenIndex % 3 + 3) % 3];
        }

        public static Motif fromAprilTag(AprilTagDetection tag) {
            return fromGreenIndex(tag.id);
        }

        /**
         * @param classifierRamp    The artifacts currently in the classifier ramp (starting from the gate)
         * @return                  The motif pattern to score, {@link null} if classifier is full
         */
        public Motif getEffectiveMotif(Artifact... classifierRamp) {

            int i = indexOf(EMPTY, classifierRamp);
            if (i == -1) return null;

            return Motif.fromGreenIndex((getGreenIndex() - i) % 3);
        }
    }

    public static class ScoringInstructions {

        /**
         * Run spindexer counter-clockwise (CCW) when scoring motif
         */
        public final boolean counterClockwise;

        /**
         * Spindexer slot index of the first artifact to score
         */
        public final int firstArtifactIndex;

        /**
         * How many more artifacts to score after the first one ({@link #firstArtifactIndex}) <br>
         * Can be 0, 1, or 2
         */
        public final int additionalArtifacts;

        private ScoringInstructions(
                boolean counterClockwise,
                int firstArtifactIndex,
                int additionalArtifacts
        ) {
            this.counterClockwise = counterClockwise;
            this.firstArtifactIndex = firstArtifactIndex;
            this.additionalArtifacts = additionalArtifacts;
        }
    }

    public static int countOf(Artifact color, Artifact... artifacts) {
        int count = 0;
        for (Artifact slot : artifacts)
            if (slot == color)
                count++;
        return count;
    }

    public static int indexOf(Artifact color, Artifact... artifacts) {
        int index = -1, length = artifacts.length;
        for (int i = 0; i < length; i++)
            if (artifacts[i] == color) {
                index = i;
                break;
            }
        return index;
    }

    public static ScoringInstructions getScoringInstructions(Motif effectiveMotif, Artifact[] spindexerSlots) {

        // indices of spindexer slots
        List<Integer> slotIndices = Arrays.asList(0, 1, 2);

        // the different colors in the provided motif
        Artifact
                m0 = effectiveMotif.artifacts[0],
                m1 = effectiveMotif.artifacts[1],
                m2 = effectiveMotif.artifacts[2];

        // find index of first artifact color needed for motif
        int firstArtifactIndex = indexOf(m0, spindexerSlots);

        // we don't have the first color needed, continue intaking
        if (firstArtifactIndex == -1) return null;

        // rotate indices so the index of the first color needed is in the first slot
        Collections.rotate(slotIndices, -firstArtifactIndex);

        // if the next artifact (clockwise from the first one) is NOT the right motif color,
        // and the third IS the right color, run the spindexer the other way
        boolean counterClockwise = spindexerSlots[slotIndices.get(1)] != m1 && spindexerSlots[slotIndices.get(2)] == m1;
        if (counterClockwise)
            Collections.swap(slotIndices, 1, 2);

        // check if we have the right second and third artifacts to complete the motif
        // if we reversed the spindexer earlier, we know we have the second artifact needed, so
        // skip that check with a short circuit OR ||
        int additionalArtifacts = 0;
        if (counterClockwise || spindexerSlots[slotIndices.get(1)] == m1) additionalArtifacts++;
        if (spindexerSlots[slotIndices.get(2)] == m2) additionalArtifacts++;

        return new ScoringInstructions(
                counterClockwise,
                firstArtifactIndex,
                additionalArtifacts
        );
    }
}
