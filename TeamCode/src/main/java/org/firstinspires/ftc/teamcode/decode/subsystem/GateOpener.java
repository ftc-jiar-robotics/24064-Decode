package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_GATE_OPENER_RED_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_GATE_OPENER_BLUE_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.util.CachedServo;

@Configurable
public class GateOpener extends Subsystem<GateOpener.GateOpenerStates> {
    private final CachedServo gateRed;
    private final CachedServo gateBlue;

    public enum GateOpenerStates {
        OPEN, CLOSE, AUTOMATIC
    }

    private GateOpenerStates currentState = GateOpenerStates.CLOSE;

    public static double
            CLOSED_ANGLE_RED = 50,
            MID_ANGLE_RED = 145,
            OPEN_ANGLE_RED = 185,
            CLOSED_ANGLE_BLUE = 235,
            OPEN_ANGLE_BLUE = 95,
            MID_ANGLE_BLUE = 140,
            GATE_OPEN_Y_MIN = 40,
            GATE_OPEN_Y_MAX = 90,
            GATE_OPEN_X_MIN_BLUE = 0,
            GATE_OPEN_X_MAX_BLUE = 20,
            GATE_OPEN_X_MIN_RED = 144-GATE_OPEN_X_MAX_BLUE,
            GATE_OPEN_X_MAX_RED = 144-GATE_OPEN_X_MIN_BLUE,
            GATE_OPEN_ANGLE_MIN_BLUE = 90,
            GATE_OPEN_ANGLE_MAX_BLUE = 180,
            GATE_OPEN_ANGLE_MIN_RED = 0,
            GATE_OPEN_ANGLE_MAX_RED = 90;

    private boolean isOpen = false;

    public GateOpener(HardwareMap hw) {
        gateRed  = new CachedServo(hw, NAME_GATE_OPENER_RED_SERVO, Common.SERVO_25_KG_MIN, Common.SERVO_25_KG_MAX, AngleUnit.DEGREES);
        gateBlue = new CachedServo(hw, NAME_GATE_OPENER_BLUE_SERVO, Common.SERVO_25_KG_MIN, Common.SERVO_25_KG_MAX, AngleUnit.DEGREES);
    }

    @Override
    protected void set(GateOpenerStates state) {
        currentState = state;
    }

    @Override
    public GateOpenerStates get() {
        return currentState;
    }

    private boolean inGateZone() {
        double x = robot.drivetrain.getPose().getX();
        double y = robot.drivetrain.getPose().getY();
        double heading = Math.toDegrees(robot.drivetrain.getHeading());
        double xMin = Common.isRed ? GATE_OPEN_X_MIN_RED : GATE_OPEN_X_MIN_BLUE;
        double xMax = Common.isRed ? GATE_OPEN_X_MAX_RED : GATE_OPEN_X_MAX_BLUE;
        double degMin = Common.isRed ? GATE_OPEN_ANGLE_MIN_RED : GATE_OPEN_ANGLE_MIN_BLUE;
        double degMax = Common.isRed ? GATE_OPEN_ANGLE_MAX_RED : GATE_OPEN_ANGLE_MAX_BLUE;

        boolean inRange = x >= xMin && x <= xMax && y >= GATE_OPEN_Y_MIN && y <= GATE_OPEN_Y_MAX;
        boolean facingWall = heading >= degMin && heading <= degMax;

        return inRange && facingWall;
    }

    private void updateOpener() {
        if (Common.isRed) {
            gateRed.turnToAngle(isOpen ? (currentState == GateOpenerStates.AUTOMATIC ? MID_ANGLE_RED : OPEN_ANGLE_RED) : CLOSED_ANGLE_RED);
            gateBlue.turnToAngle(CLOSED_ANGLE_BLUE);
        } else {
            gateBlue.turnToAngle(isOpen ? (currentState == GateOpenerStates.AUTOMATIC ? MID_ANGLE_BLUE : OPEN_ANGLE_BLUE) : CLOSED_ANGLE_BLUE);
            gateRed.turnToAngle(CLOSED_ANGLE_RED);
        }
    }

    @Override
    public void run() {
        switch (currentState) {
            case OPEN:
                isOpen = true;
                break;
            case CLOSE:
                isOpen = false;
                break;
            case AUTOMATIC:
                isOpen = inGateZone();
                break;
        }

        updateOpener();
    }

    @Override
    public void printTelemetry() {
        telemetry.addLine("GATE OPENER");
        telemetry.addData("state (enum): ", currentState);
        telemetry.addData("is open (BOOLEAN): ", isOpen);
        telemetry.addData("in gate zone (BOOLEAN): ", inGateZone());
    }
}