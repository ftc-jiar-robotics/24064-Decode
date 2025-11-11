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
        public final double greenArtifactRadians;

        public final Artifact a0, a1, a2;

        private static final Motif[] motifs = values();

        Motif(double greenArtifactRadians, Artifact a0, Artifact a1, Artifact a2) {
            this.greenArtifactRadians = greenArtifactRadians;
            this.a0 = a0;
            this.a1 = a1;
            this.a2 = a2;
        }

        public Artifact[] toArray() {
            return new Artifact[]{a0, a1, a2};
        }

        public int getGreenIndex() {
            return ordinal();
        }

        public static Motif fromArray(Artifact... artifacts) {
            assert artifacts.length == motifs.length;
            return fromGreenIndex(indexOf(GREEN, artifacts));
        }

        public static Motif fromGreenIndex(int greenIndex) {
            int length = motifs.length;
            return motifs[(greenIndex % length + length) % length];
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

            return Motif.fromGreenIndex((getGreenIndex() - i) % motifs.length);
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

        public String toString() {
            return String.format("Rotating %s, score slot %s artifact first, then score %s more artifacts", counterClockwise ? "CCW" : "CW", firstArtifactIndex, additionalArtifacts);
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

    /**
     * @param effectiveMotif    THIS MUST BE THE LATEST OUTPUT FROM randomization.getEffectiveMotif(classifierState)
     * @param spindexerSlots    Artifacts available in the spindexer (0 = front, 1 = back left, 2 = back right)
     * @return                  Instructions for spindexer to score artifacts in the correct order, {@link null} if we don't have the artifacts needed for the motif
     */
    public static ScoringInstructions getScoringInstructions(Motif effectiveMotif, Artifact... spindexerSlots) {

        // find index of first artifact color needed for motif
        int firstArtifactIndex = indexOf(effectiveMotif.a0, spindexerSlots);

        // we don't have the first color needed, continue intaking
        if (firstArtifactIndex == -1) return null;

        Artifact secondArtifact = spindexerSlots[(firstArtifactIndex + 1) % spindexerSlots.length];
        Artifact thirdArtifact = spindexerSlots[(firstArtifactIndex + 2) % spindexerSlots.length];

        // if the next artifact (clockwise from the first one) is NOT the  next motif color,
        // and the third IS the next motif color, run the spindexer the other way
        boolean counterClockwise = secondArtifact != effectiveMotif.a1 && thirdArtifact == effectiveMotif.a1;
        if (counterClockwise) thirdArtifact = secondArtifact;

        return new ScoringInstructions(
                counterClockwise,
                firstArtifactIndex,
                (counterClockwise || secondArtifact == effectiveMotif.a1 ? 1 : 0) + (thirdArtifact == effectiveMotif.a2 ? 1 : 0)
        );
    }
}
