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
            CLOSED_ANGLE_RED = 50,
            OPEN_ANGLE_RED = 170,
            CLOSED_ANGLE_BLUE = 230,
            OPEN_ANGLE_BLUE = 110;

    public GateOpener(HardwareMap hw) {
        gateRed  = new CachedServo(hw, NAME_GATE_OPENER_RED_SERVO, Common.SERVO_25_KG_MIN, Common.SERVO_25_KG_MAX, AngleUnit.DEGREES);
        gateBlue = new CachedServo(hw, NAME_GATE_OPENER_BLUE_SERVO, Common.SERVO_25_KG_MIN, Common.SERVO_25_KG_MAX, AngleUnit.DEGREES);
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
            gateRed.turnToAngle(isOpen ? OPEN_ANGLE_RED : CLOSED_ANGLE_RED);
            gateBlue.turnToAngle(CLOSED_ANGLE_BLUE);
        } else {
            gateBlue.turnToAngle(isOpen ? OPEN_ANGLE_BLUE : CLOSED_ANGLE_BLUE);
            gateRed.turnToAngle(CLOSED_ANGLE_RED);
        }
    }

    @Override
    public void printTelemetry() {
        telemetry.addLine("GATE OPENER");
        telemetry.addData("is open (BOOLEAN): ", isOpen);
    }
}