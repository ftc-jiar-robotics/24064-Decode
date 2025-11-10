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

    public static double toleranceRadians = toRadians(5);
    public static boolean inTolerance(double currentRadians, double targetRadians) {
        return abs(targetRadians - currentRadians) <= toleranceRadians;
    }

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

        public static Artifact fromHSV(HSV hsv) {
            return
                    hsv.between(minPurple, maxPurple) ? PURPLE :
                    hsv.between(minGreen, maxGreen) ? GREEN :
                    EMPTY;
        }
    }

    private final ColorSensor colorSensor;
    private final AnalogEncoder encoder;
    private final CRServo[] servos = new CRServo[2];
    private void setServos(double power) {
        for (CRServo servo : servos) servo.setPower(power);
    }

    private final PIDController controller = new PIDController();
    private final Artifact[] slots = {EMPTY, EMPTY, EMPTY};
    private final Artifact[] motif = {PURPLE, PURPLE, GREEN};

    private int targetFrontSlot = 0;
    private int changeTargetFrontSlot(int delta) {
        return targetFrontSlot = (targetFrontSlot + delta) % 3;
    }
    private double currentRadians = 0;
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
        currentRadians = normalizeRadians(encoder.getPosition());

        switch (state) {

            case PASSTHROUGH:

                // TODO Remove this call and call it conditionally elsewhere (ex. only when intaking)
                updateColorSensor();

                runServosUsingPID();

                break;

            case INDEXING:

                // TODO Remove this call and call it conditionally elsewhere (ex. only when intaking)
                updateColorSensor();

                if (hasMotifArtifacts()) {
                    state = State.PREPARING_MOTIF;
                    //TODO find index of first motif color, need alg
//                    targetFrontSlot = firstMotifColor(); something something
                } else if (frontSlotHasArtifact())
                    changeTargetFrontSlot(1);

                runServosUsingPID();

                break;

            case PREPARING_MOTIF:

                runServosUsingPID();

                break;

            case SHOOTING_MOTIF:

                setServos(0);

                break;
        }
    }

    private void runServosUsingPID() {
        controller.setTarget(new org.firstinspires.ftc.teamcode.decode.control.motion.State(normalizeRadians(getTargetRadians(targetFrontSlot) - currentRadians) + currentRadians));
        setServos(controller.calculate(new org.firstinspires.ftc.teamcode.decode.control.motion.State(currentRadians)));
    }

    public void updateColorSensor() {
        // dont save color value to next index spot accidentally
        if (!inTolerance(currentRadians, getTargetRadians(targetFrontSlot))) return;

        colorSensor.update();
        slots[targetFrontSlot] = Artifact.fromHSV(colorSensor.hsv);
    }

    public boolean frontSlotHasArtifact() {
        return slots[targetFrontSlot] != EMPTY;
    }

    public int count(Artifact color) {
        int count = 0;
        for (Artifact slot : slots)
            if (slot == color)
                count++;
        return count;
    }

    public boolean hasMotifArtifacts() {
        return count(PURPLE) == 2 && count(GREEN) == 1;
    }

    public boolean firstMotifArtifactReady() {
        return state == State.PREPARING_MOTIF && inTolerance(currentRadians, getTargetRadians(targetFrontSlot));
    }

    public static double getTargetRadians(int slot) {
        return slot * PI / 3.0;
    }

    @Override
    public void printTelemetry() {
        telemetry.addLine(colorSensor.hsv.toString("Spindexer Color Sensor [" + slots[targetFrontSlot].name() + "]"));
    }


}
