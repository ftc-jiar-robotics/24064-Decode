package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.HSV;

@Configurable
public enum Artifact {
    PURPLE,
    GREEN,
    EMPTY;

    public static final HSV
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

    public static Artifact fromHSV(HSV hsv) {
        return
                hsv.between(minPurple, maxPurple) ? PURPLE :
                        hsv.between(minGreen, maxGreen) ? GREEN :
                                EMPTY;
    }

    /**
     * @return The number of times this artifact color appears in the provided array
     */
    public int countOccurrencesIn(Artifact... artifacts) {
        int count = 0;
        for (Artifact artifact : artifacts)
            if (artifact == this)
                count++;
        return count;
    }

    /**
     * @return The index of the first occurrence of this artifact color in the provided array, -1 if no occurrences
     */
    public int firstOccurrenceIn(Artifact... artifacts) {
        for (int i = 0; i < artifacts.length; i++)
            if (artifacts[i] == this)
                return i;
        return -1;
    }
}
