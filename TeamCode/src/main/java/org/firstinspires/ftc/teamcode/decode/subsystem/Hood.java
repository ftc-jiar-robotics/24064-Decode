package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Hood extends Subsystem<Double> {
    private final SimpleServo hood;

    private double targetAngle;

    public Hood(HardwareMap hw) {
        this.hood = new SimpleServo(hw, "hood", Common.SERVO_AXON_MIN, Common.SERVO_AXON_MAX_1);
    }

    @Override
    public Double get() {
        return targetAngle;
    }

    @Override
    public void set(Double a) {
        targetAngle = a;
    }

    @Override
    public void run() {
        hood.turnToAngle(targetAngle);
    }

    public void printTelemetry() {
        telemetry.addData("hood target angle: ", targetAngle);
    }
}