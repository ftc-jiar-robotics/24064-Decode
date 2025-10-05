package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.BACKWARD;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;

@Configurable
public class Feeder extends Subsystem<Feeder.FeederStates> {

    // TODO make isShooterReady based whether odometry hits triangle // isTracking // shooter queue counter != 0
    // TODO if manual, then ignore isTracking

    public final CRServo feederFront;
    public final CRServo feederBack;

    private final ColorSensor colorSensor;

    private FeederStates currentState = FeederStates.OFF;

    private float gain = 0; //TODO: change gain

    public enum FeederStates {
        OFF, OUTTAKING, IDLE, RUNNING
    }

    public Feeder(HardwareMap hw) {
        feederFront = hw.get(CRServo.class, Common.CFG_NAME_FEEDERFRONT);
        feederBack = hw.get(CRServo.class, Common.CFG_NAME_FEEDERBACK);
        this.colorSensor = new ColorSensor(hw, Common.CFG_NAME_FEEDER_COLORSENSOR, gain);

        feederFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void set(FeederStates state) {
        currentState = state;
    }

    @Override
    public FeederStates get() {
        return currentState;
    }

    @Override
    public void run() {
            switch (currentState) {
                case OFF: {
                    feederFront.setPower(0);
                    feederBack.setPower(0);
                    break;
                }
                case OUTTAKING: {
                    feederFront.setPower(1);
                    feederBack.setPower(-1);
                    break;
                }
                case IDLE: {
                    feederFront.setPower(-1);
                    feederBack.setPower(-1);
                    break;
                }
                case RUNNING: {
                    feederFront.setPower(-1);
                    feederBack.setPower(1);
                    break;
                }
            }
    }

    public void printTelemetry() {
        telemetry.addLine("FEEDER");
        telemetry.addData("current state: ", currentState);
    }
}
