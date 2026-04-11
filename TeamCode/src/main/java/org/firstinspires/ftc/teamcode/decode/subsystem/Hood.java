package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_HOOD_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
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
            MAX = 160,
            MIN = 80;


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

        // in launch radians: 1.03332039 - (0.00620188 * distance) + (0.00001780 * distance * distance);
    }
    public double getHoodAngleWithRPM(double currentRPM) {
        double angle = -87.51927299612511*(1) + 0.1366987334209007*(currentRPM) + -1.888596858075093e-05*(currentRPM*currentRPM);
        return Range.clip(angle, MIN, MAX);

        // in launch radians: 1.46808546 - (0.00054638 * currentRPM) + (0.0000000755 * currentRPM * currentRPM);
        }

    public double launchRadiansToServoAngle(double targetRadians) {
        double scaleFactor = (Hood.MAX - Hood.MIN) / (Math.toDegrees(KinematicsSolver.θ_launchMax) - Math.toDegrees(KinematicsSolver.θ_launchMin));

        return Hood.MIN + Math.toDegrees(KinematicsSolver.θ_launchMax - targetRadians) * scaleFactor;
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