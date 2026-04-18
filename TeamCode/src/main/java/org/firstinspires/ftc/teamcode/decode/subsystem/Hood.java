package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_HOOD_SERVO;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Flywheel.GEAR_RATIO;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.util.CachedServo;

import java.util.TreeMap;

@Configurable
public class Hood extends Subsystem<Double> {
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
        // in launch radians: 1.03332039 - (0.00620188 * distance) + (0.00001780 * distance * distance);
        return Range.clip(110.35209094986163*(1) + -1.134249848424577*(distance) + 0.011055576786071695*(distance*distance) + -1.071867716505115e-05*(distance*distance*distance), MIN, MAX);
    }

    public double getHoodAngleWithRPM(double currentRPM) {
        currentRPM *= 1 / GEAR_RATIO;
        double angle = 633.1781464272758*(1) + -0.8104906629617021*(currentRPM) + 0.0003741537288974748*(currentRPM*currentRPM) + -5.2131442194449443e-08*(currentRPM*currentRPM*currentRPM);
//        if (robot.isFar) angle += 15;
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