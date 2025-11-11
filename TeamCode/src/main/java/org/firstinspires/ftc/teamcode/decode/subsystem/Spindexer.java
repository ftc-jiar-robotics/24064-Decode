package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Spindexer.Artifact.EMPTY;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Spindexer.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Spindexer.Artifact.PURPLE;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.PIDGains;
import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;
import org.firstinspires.ftc.teamcode.decode.util.AnalogEncoder;

@Configurable
public final class Spindexer extends Subsystem<Spindexer.State> {

    public static double RADIANS_TOLERANCE = toRadians(5);

    public static PIDGains gains = new PIDGains(0, 0, 0);

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

    public enum State {
        PASSTHROUGH,
        INDEXING,
        PREPARING_MOTIF,
        SHOOTING_MOTIF,
    }

    public enum Artifact {
        PURPLE,
        GREEN,
        EMPTY;

        private static Artifact fromHSV(HSV hsv) {
            return
                    hsv.between(minPurple, maxPurple) ? PURPLE :
                    hsv.between(minGreen, maxGreen) ? GREEN :
                    EMPTY;
        }
    }

    public enum Motif {
        GPP(PI/6,   GREEN, PURPLE, PURPLE),
        PGP(PI,     PURPLE, GREEN, PURPLE),
        PPG(-PI/6,  PURPLE, PURPLE, GREEN);

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

        public int toGreenIndex() {
            return ordinal();
        }

        public static Motif fromArray(Artifact... artifacts) {
            return fromGreenIndex(indexOf(GREEN, artifacts));
        }

        public static Motif fromGreenIndex(int greenIndex) {
            return motifs[greenIndex];
        }


    }

    // hardware
    private final ColorSensor colorSensor;
    private final AnalogEncoder encoder;
    private final CRServo[] servos = new CRServo[2];
    private void setServos(double power) {
        for (CRServo servo : servos) servo.setPower(power);
    }

    private final PIDController controller = new PIDController();

    // spindexer state
    private final Artifact[] slots = {EMPTY, EMPTY, EMPTY};
    private double currentSlot0Radians = 0; // physical position of slot 0
    private double currentSlotRadians(int slot) {
        return currentSlot0Radians + slot * 2 * PI / 3.0;
    }
    private State state = State.PASSTHROUGH;

    Spindexer(HardwareMap hardwareMap) {
        colorSensor = new ColorSensor(hardwareMap, "spindexer color", 1.0f);

        servos[0] = hardwareMap.get(CRServo.class, "spindexer 1");
        servos[1] = hardwareMap.get(CRServo.class, "spindexer 2");

        encoder = new AnalogEncoder(hardwareMap, "spindexer encoder", 3 * 2 * PI);
    }

    public State get() {
        return state;
    }

    protected void set(State state) {
        this.state = state;
    }

    public void run() {

        // update currentAngle with encoder readings
        currentSlot0Radians = normalizeRadians(encoder.getPosition());

        switch (state) {

            case PASSTHROUGH:

                break;

            case INDEXING:

                break;

            case PREPARING_MOTIF:

                break;

            case SHOOTING_MOTIF:

                break;
        }


    }

    /**
     * Set PID target to rotate Spindexer until a slot is at a particular position
     * @param slot          Slot you want to move
     * @param targetRadians Where you want to move it to
     */
    private void runPID(int slot, double targetRadians) {
        double setpoint = currentSlot0Radians + getError(slot, targetRadians);

        controller.setTarget(new org.firstinspires.ftc.teamcode.decode.control.motion.State(setpoint));
        setServos(controller.calculate(new org.firstinspires.ftc.teamcode.decode.control.motion.State(currentSlot0Radians)));
    }

    /**
     * @param slot          Slot whose position you are querying
     * @param targetRadians Where it should be
     * @return              If the given slot is at the given target, within {@link #RADIANS_TOLERANCE}
     */
    private boolean slotIsAtPosition(int slot, double targetRadians) {
        return abs(getError(slot, targetRadians)) <= RADIANS_TOLERANCE;
    }

    /**
     * @param slot          Slot whose position you are querying
     * @param targetRadians Where the slot should be
     * @return              Distance, in radians, between given slot's position and given targetRadians
     */
    private double getError(int slot, double targetRadians) {
        return normalizeRadians(targetRadians - currentSlotRadians(slot));
    }

    public void updateColorSensor() {
        int slot;

        if      (slotIsAtPosition(0, 0)) slot = 0;
        else if (slotIsAtPosition(1, 0)) slot = 1;
        else if (slotIsAtPosition(2, 0)) slot = 2;
        else return;

        colorSensor.update();
        slots[slot] = Artifact.fromHSV(colorSensor.hsv);
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

    public void printTelemetry() {
        telemetry.addLine(colorSensor.hsv.toString(String.format("Spindexer Color Sensor [%s]", Artifact.fromHSV(colorSensor.hsv))));
    }


}
