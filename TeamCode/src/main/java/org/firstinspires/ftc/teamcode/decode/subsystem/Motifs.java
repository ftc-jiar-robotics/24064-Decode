package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Motifs.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Motifs.Artifact.PURPLE;
import static java.lang.Math.PI;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.HSV;

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
            return fromGreenIndex(indexOf(GREEN, artifacts));
        }

        public static Motif fromGreenIndex(int greenIndex) {
            return motifs[greenIndex];
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

    public static int[] reverseAt(int center, int[] array) {
        assert array.length == 3;
        int[] ints = array.clone();
        int j = (center + 1) % 3;
        int k = (j + 1) % 3;
        int valJ = ints[j];
        ints[j] = ints[k];
        ints[k] = valJ;
        return ints;
    }
}
