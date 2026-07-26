package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_PARK_LIFT_LEFT_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_PARK_LIFT_RIGHT_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.util.CachedServo;

@Configurable
public class ParkLift extends Subsystem<ParkLift.ParkStates> {
    private final CachedServo parkRight;
    private final CachedServo parkLeft;

    public enum ParkStates {
        UP, DOWN, RESET
    }
    private ParkLift.ParkStates currentState = ParkLift.ParkStates.UP;

    public static double
            UP_ANGLE_RIGHT = 170,
            DOWN_ANGLE_RIGHT = 60,
            UP_ANGLE_LEFT = 170,
            DOWN_ANGLE_LEFT = 60;


    public ParkLift(HardwareMap hw) {
        parkLeft  = new CachedServo(hw, NAME_PARK_LIFT_LEFT_SERVO, Common.SERVO_AXON_MIN, Common.SERVO_AXON_MAX_2, AngleUnit.DEGREES);
        parkRight = new CachedServo(hw, NAME_PARK_LIFT_RIGHT_SERVO, Common.SERVO_AXON_MIN, Common.SERVO_AXON_MAX_2, AngleUnit.DEGREES);

        parkLeft.setInverted(true);
    }
    @Override
    protected void set(ParkLift.ParkStates state) {
        currentState = state;
    }

    @Override
    public ParkLift.ParkStates get() {
        return currentState;
    }

    @Override
    public void run() {
        switch (currentState) {
            case DOWN:
                parkLeft.turnToAngle(DOWN_ANGLE_LEFT);
                parkRight.turnToAngle(DOWN_ANGLE_RIGHT);
                if (robot != null) {
                    robot.shooter.clearQueueShots();
                }
                break;
            case UP:
                parkLeft.turnToAngle(UP_ANGLE_LEFT);
                parkRight.turnToAngle(UP_ANGLE_RIGHT);
                break;
            case RESET:
                parkLeft.turnToAngle(0);
                parkRight.turnToAngle(0);
        }
    }



    @Override
    public void printTelemetry() {
        telemetry.addLine("Park Lifty");
        telemetry.addData("state (enum): ", currentState);
    }
}
