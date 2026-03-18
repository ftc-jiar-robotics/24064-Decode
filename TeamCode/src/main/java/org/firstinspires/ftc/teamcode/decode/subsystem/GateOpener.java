package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_GATE_OPENER_RED_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_GATE_OPENER_BLUE_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.util.CachedServo;

@Configurable
public class GateOpener extends Subsystem<Boolean> {
    private final CachedServo gateRed;
    private final CachedServo gateBlue;
    private boolean isOpen = false;

    public static double
            CLOSED_ANGLE = 0,
            OPEN_ANGLE = 90;

    public GateOpener(HardwareMap hw) {
        gateRed  = new CachedServo(hw, NAME_GATE_OPENER_RED_SERVO, Common.SERVO_AXON_MIN, Common.SERVO_AXON_MAX_2, AngleUnit.DEGREES);
        gateBlue = new CachedServo(hw, NAME_GATE_OPENER_BLUE_SERVO, Common.SERVO_AXON_MIN, Common.SERVO_AXON_MAX_2, AngleUnit.DEGREES);
    }

    @Override
    protected void set(Boolean open) {
        isOpen = open;
    }

    @Override
    public Boolean get() {
        return isOpen;
    }

    @Override
    public void run() {
        if (Common.isRed) {
            gateRed.turnToAngle(isOpen ? OPEN_ANGLE : CLOSED_ANGLE);
            gateBlue.turnToAngle(CLOSED_ANGLE);
        } else {
            gateBlue.turnToAngle(isOpen ? OPEN_ANGLE : CLOSED_ANGLE);
            gateRed.turnToAngle(CLOSED_ANGLE);
        }
    }

    @Override
    public void printTelemetry() {
        telemetry.addLine("GATE OPENER");
        telemetry.addData("is open (BOOLEAN): ", isOpen);
    }
}