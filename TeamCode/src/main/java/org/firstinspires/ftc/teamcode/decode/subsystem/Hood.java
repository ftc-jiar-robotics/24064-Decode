package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_HOOD_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.util.CachedServo;

import java.util.TreeMap;

@Configurable
public class    Hood extends Subsystem<Double> {
    private final CachedServo hood;

    private double
            targetAngle,
            lutOutput;

    public static double
            PHYSICAL_MAX = 195,
            MAX = 180,
            MIN = 90;


    public Hood(HardwareMap hw) {
        this.hood = new CachedServo(hw, NAME_HOOD_SERVO, Common.SERVO_AXON_MIN, Common.SERVO_AXON_MAX_2, AngleUnit.DEGREES);
    }

    @Override
    public Double get() {
        return targetAngle;
    }

    @Override
    public void set(Double a) {
        targetAngle = Range.clip(a, MIN, MAX);
    }

    public void setPhysicalMax() {
        targetAngle = PHYSICAL_MAX;
    }

    public double getHoodAngleWithDistance(double distance) {

        return Range.clip(21.25528184671319*(1) + 1.551657839403464*(distance) + -0.00445309317429351*(distance*distance), MIN, MAX);
    }
    public double getHoodAngleWithRPM(double currentRPM) {
        double angle = -87.51927299612511*(1) + 0.1366987334209007*(currentRPM) + -1.888596858075093e-05*(currentRPM*currentRPM);
        return Range.clip(angle, MIN, MAX);
    }

    @Override
    public void run() {
        hood.turnToAngle(Range.clip(targetAngle, MIN, PHYSICAL_MAX));
    }

    public void printTelemetry() {
        telemetry.addLine("HOOD");
        telemetry.addData("hood target angle (ANGLE): ", targetAngle);
        telemetry.addData("LUT output (INCHES): ", lutOutput);

    }
}