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
            PHYSICAL_MAX = 200,
            MAX = 180,
            MIN = 68;


    public Hood(HardwareMap hw) {
        this.hood = new CachedServo(hw, NAME_HOOD_SERVO, Common.SERVO_AXON_MIN, Common.SERVO_AXON_MAX_1, AngleUnit.DEGREES);
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

        return Range.clip(10+26.182014839692346*(1) + 1.1213299138282586*(distance) + -0.002162724921235206*(distance*distance), MIN, MAX);
    }
    public double getHoodAngleWithRPM(double currentRPM) {
        double angle = 5+-252.051601447441*(1) + 0.21523829719632345*(currentRPM) + -2.842690046324649e-05*(currentRPM * currentRPM);
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